// World snapshots: a stack of boxes still settling, its world saved mid settle and restored into a
// second world, and both stepped on. Every body ends every step at the same bits in both, and a
// world rebuilt from the bodies' positions alone (no contacts, no warm starting) does not, which is
// what the snapshot carries that the positions do not. Then a restore into the world the snapshot
// came from puts it back to that step. Run against every build.

import assert from 'node:assert/strict';

const DT = 1 / 60;

function stack( b3 )
{
	const def = b3.b3DefaultWorldDef();
	def.gravity = [ 0, -10, 0 ];
	const world = b3.b3CreateWorld( def );
	const groundDef = b3.b3DefaultBodyDef();
	groundDef.type = b3.b3BodyType.b3_staticBody;
	b3.b3CreateBoxShape( b3.b3CreateBody( world, groundDef ), b3.b3DefaultShapeDef(), 25, 0.5, 25 );
	const bodies = [];
	for ( let i = 0; i < 4; i++ )
	{
		const bd = b3.b3DefaultBodyDef();
		bd.type = b3.b3BodyType.b3_dynamicBody;
		bd.position = [ 0.07 * i, 1.0 + 1.05 * i, -0.05 * i ];
		const body = b3.b3CreateBody( world, bd );
		b3.b3CreateBoxShape( body, b3.b3DefaultShapeDef(), 0.5, 0.5, 0.5 );
		bodies.push( body );
	}
	return { world, bodies };
}

// Every body's position and rotation, as exact numbers.
function state( b3, bodies )
{
	return bodies.map( b => [ ...b3.b3Body_GetPosition( [ 0, 0, 0 ], b ), ...b3.b3Body_GetRotation( [ 0, 0, 0, 1 ], b ) ] );
}

// The body ids of a world restored into another, which name the same slots in that world.
const into = ( world, bodies ) => bodies.map( b => ( { ...b, world0: world.index1 - 1 } ) );

async function check( label, importPath )
{
	const { default: Box3D } = await import( importPath );
	const b3 = await Box3D();

	const a = stack( b3 );
	for ( let i = 0; i < 25; i++ ) b3.b3World_Step( a.world, DT, 4 );
	const image = b3.b3World_GetSnapshot( a.world );
	assert.ok( image instanceof Uint8Array && image.length > 16, `${label}: a snapshot image` );

	// A second world, of other bodies, restored from the image.
	const other = stack( b3 );
	assert.equal( b3.b3World_Restore( other.world, image ), true, `${label}: the image restores` );
	const b = { world: other.world, bodies: into( other.world, a.bodies ) };
	assert.deepEqual( state( b3, b.bodies ), state( b3, a.bodies ), `${label}: restored where it was saved` );

	// A third, rebuilt from the positions alone.
	const c = stack( b3 );
	a.bodies.forEach( ( body, i ) =>
	{
		const [ x, y, z, qx, qy, qz, qw ] = state( b3, [ body ] )[ 0 ];
		b3.b3Body_SetTransform( c.bodies[ i ], [ x, y, z ], [ qx, qy, qz, qw ] );
		b3.b3Body_SetLinearVelocity( c.bodies[ i ], b3.b3Body_GetLinearVelocity( [ 0, 0, 0 ], body ) );
		b3.b3Body_SetAngularVelocity( c.bodies[ i ], b3.b3Body_GetAngularVelocity( [ 0, 0, 0 ], body ) );
	} );

	let parted = 0;
	for ( let i = 0; i < 120; i++ )
	{
		b3.b3World_Step( a.world, DT, 4 );
		b3.b3World_Step( b.world, DT, 4 );
		b3.b3World_Step( c.world, DT, 4 );
		assert.deepEqual( state( b3, b.bodies ), state( b3, a.bodies ), `${label}: restored world steps alike, step ${i}` );
		if ( JSON.stringify( state( b3, c.bodies ) ) !== JSON.stringify( state( b3, a.bodies ) ) ) parted++;
	}
	assert.ok( parted > 0, `${label}: a world rebuilt from positions alone steps otherwise` );

	// Restoring the world the image came from puts it back to that step.
	const later = state( b3, a.bodies );
	assert.equal( b3.b3World_Restore( a.world, image ), true );
	const back = state( b3, a.bodies );
	assert.notDeepEqual( back, later );
	for ( let i = 0; i < 120; i++ ) b3.b3World_Step( a.world, DT, 4 );
	assert.deepEqual( state( b3, a.bodies ), later, `${label}: a world restored in place steps to the same place again` );

	assert.equal( b3.b3World_Restore( a.world, new Uint8Array( 20 ) ), false, `${label}: an image that is not one is refused` );

	for ( const w of [ a.world, other.world, c.world ] ) b3.b3DestroyWorld( w );
	console.log( `${label}: snapshot OK (rebuilt-from-positions parted on ${parted} of 120 steps)` );
}

await check( 'separate', '../dist/box3d.mjs' );
await check( 'inline', '../dist/box3d.inline.mjs' );
