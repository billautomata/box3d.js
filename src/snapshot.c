// World snapshots for box3d.js: the whole of a live world as one self-contained image, and that image
// put back into a world, as Box2D's b2World_GetSnapshot and b2World_Restore do.
//
// box3d already serializes a world for its recordings (src/world_snapshot.c): bodies, shapes,
// contacts with their manifolds and warm starting impulses, joints, sensors, islands, solver sets,
// the broad phase and the constraint graph. It writes hull, mesh, height field and compound geometry
// into a recording's registry rather than into the image, so here the image is the world's snapshot
// followed by that registry:
//   u32 magic 'B3WI', u32 version, u32 snapshot bytes, u32 registry bytes, snapshot, registry
// A world restored from one steps on exactly as the world it was taken from, given the same calls.
//
// A restored mesh, height field or compound shape points straight at the geometry bytes the image
// carried, so those bytes are kept for the world (`held`, by world slot) and let go when the world is
// restored again or destroyed (b3World_ReleaseSnapshot, which the bindings call from b3DestroyWorld).
// A hull is copied into the world and needs nothing kept.

#include "physics_world.h"
#include "recording.h"
#include "recording_replay.h"
#include "world_snapshot.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"

#include <string.h>

#define B3_IMAGE_MAGIC 0x49573342u // 'B3WI'
#define B3_IMAGE_VERSION 1u

typedef struct b3Held
{
	b3WorldId world;
	b3RegistrySlot* slots;
	int slotCount;
} b3Held;

static b3Held held[B3_MAX_WORLDS];

static uint32_t b3ReadU32( const uint8_t* p )
{
	return (uint32_t)p[0] | ( (uint32_t)p[1] << 8 ) | ( (uint32_t)p[2] << 16 ) | ( (uint32_t)p[3] << 24 );
}

static void b3WriteU32( uint8_t* p, uint32_t v )
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)( v >> 8 );
	p[2] = (uint8_t)( v >> 16 );
	p[3] = (uint8_t)( v >> 24 );
}

static void b3FreeHeldSlots( b3RegistrySlot* slots, int slotCount )
{
	if ( slots == NULL )
	{
		return;
	}
	for ( int i = 0; i < slotCount; ++i )
	{
		b3RegistrySlot* slot = slots + i;
		// A compound is rebuilt into a copy of its bytes; a mesh and a height field use the bytes themselves.
		if ( slot->live != NULL && slot->kind == b3_geometryCompound )
		{
			b3Free( slot->live, (size_t)slot->byteCount );
		}
		if ( slot->bytes != NULL )
		{
			b3Free( slot->bytes, slot->byteCount > 0 ? (size_t)slot->byteCount : 1u );
		}
	}
	b3Free( slots, (size_t)slotCount * sizeof( b3RegistrySlot ) );
}

// The registry block as b3RecWriteRegistry writes it: u32 count, then per entry u8 kind, u32 byte
// count and the bytes. The query tag table after the entries is a recording's, and is not read.
static bool b3LoadHeldSlots( const uint8_t* p, int size, b3RegistrySlot** outSlots, int* outCount )
{
	*outSlots = NULL;
	*outCount = 0;
	if ( size < 4 )
	{
		return false;
	}
	const uint8_t* end = p + size;
	uint32_t count = b3ReadU32( p );
	p += 4;
	if ( count == 0 )
	{
		return true;
	}
	if ( (size_t)count > (size_t)( end - p ) / 5 )
	{
		return false;
	}
	b3RegistrySlot* slots = (b3RegistrySlot*)b3Alloc( (size_t)count * sizeof( b3RegistrySlot ) );
	memset( slots, 0, (size_t)count * sizeof( b3RegistrySlot ) );
	for ( uint32_t i = 0; i < count; ++i )
	{
		if ( p + 5 > end )
		{
			b3FreeHeldSlots( slots, (int)count );
			return false;
		}
		uint8_t kind = p[0];
		uint32_t byteCount = b3ReadU32( p + 1 );
		p += 5;
		if ( byteCount > (uint32_t)( end - p ) )
		{
			b3FreeHeldSlots( slots, (int)count );
			return false;
		}
		uint8_t* bytes = (uint8_t*)b3Alloc( byteCount > 0 ? (size_t)byteCount : 1u );
		if ( byteCount > 0 )
		{
			memcpy( bytes, p, (size_t)byteCount );
		}
		p += byteCount;
		slots[i].kind = (b3GeometryKind)kind;
		slots[i].byteCount = (int)byteCount;
		slots[i].bytes = bytes;
		slots[i].live = NULL;
	}
	*outSlots = slots;
	*outCount = (int)count;
	return true;
}

int b3World_GetSnapshot( b3WorldId worldId, uint8_t* image, int capacity );
bool b3World_Restore( b3WorldId worldId, const uint8_t* image, int size );
void b3World_ReleaseSnapshot( b3WorldId worldId );

// Writes the image into `image` when it fits in `capacity`, and returns its size either way, so a
// call with no buffer asks how big it is. 0 when the world is mid step.
int b3World_GetSnapshot( b3WorldId worldId, uint8_t* image, int capacity )
{
	b3World* world = b3GetWorldFromId( worldId );
	if ( world == NULL || world->locked )
	{
		return 0;
	}

	b3Recording* rec = b3CreateRecording( 0 );
	b3RecBuffer snap = { 0 };
	b3SerializeWorld( world, &snap, rec );
	rec->buffer.size = 0;
	b3RecWriteRegistry( rec );

	int size = 16 + snap.size + rec->buffer.size;
	if ( image != NULL && size <= capacity )
	{
		b3WriteU32( image, B3_IMAGE_MAGIC );
		b3WriteU32( image + 4, B3_IMAGE_VERSION );
		b3WriteU32( image + 8, (uint32_t)snap.size );
		b3WriteU32( image + 12, (uint32_t)rec->buffer.size );
		memcpy( image + 16, snap.data, (size_t)snap.size );
		memcpy( image + 16 + snap.size, rec->buffer.data, (size_t)rec->buffer.size );
	}

	b3RecBufFree( &snap );
	b3DestroyRecording( rec );
	return size;
}

// The world put back to the image: its slot, generation and host wiring (callbacks) kept, so ids
// held from before still name the world. False, with the world untouched, on an image that is not
// one, or of another build; a world whose snapshot breaks off partway is unusable.
bool b3World_Restore( b3WorldId worldId, const uint8_t* image, int size )
{
	b3World* world = b3GetWorldFromId( worldId );
	if ( world == NULL || world->locked || image == NULL || size < 16 )
	{
		return false;
	}
	if ( b3ReadU32( image ) != B3_IMAGE_MAGIC || b3ReadU32( image + 4 ) != B3_IMAGE_VERSION )
	{
		return false;
	}
	uint32_t snapSize = b3ReadU32( image + 8 );
	uint32_t regSize = b3ReadU32( image + 12 );
	if ( (uint64_t)16 + snapSize + regSize != (uint64_t)size )
	{
		return false;
	}

	b3RecReader rdr;
	memset( &rdr, 0, sizeof( rdr ) );
	rdr.replayWorldId = worldId;
	rdr.ok = true;
	if ( b3LoadHeldSlots( image + 16 + snapSize, (int)regSize, &rdr.slots, &rdr.slotCount ) == false )
	{
		return false;
	}

	bool ok = b3DeserializeIntoShell( image + 16, (int)snapSize, world, &rdr );

	// The world's shapes now point into the new slots: the old ones go.
	b3Held* h = held + worldId.index1;
	b3FreeHeldSlots( h->slots, h->slotCount );
	h->world = worldId;
	h->slots = rdr.slots;
	h->slotCount = rdr.slotCount;
	return ok;
}

// The geometry a restore kept for the world let go. Called once the world is destroyed.
void b3World_ReleaseSnapshot( b3WorldId worldId )
{
	b3Held* h = held + worldId.index1;
	if ( h->world.index1 != worldId.index1 || h->world.generation != worldId.generation )
	{
		return;
	}
	b3FreeHeldSlots( h->slots, h->slotCount );
	memset( h, 0, sizeof( b3Held ) );
}
