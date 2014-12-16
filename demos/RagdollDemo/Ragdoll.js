function RagDoll(da,positionOffset){

  var BodyParts = function() {
    this.PELVIS=null;
    this.SPINE=null;
    this.HEAD=null;
    this.LEFT_UPPER_LEG=null;
    this.LEFT_LOWER_LEG=null;
    this.LEFT_FOOT=null;
    this.RIGHT_UPPER_LEG=null;
    this.RIGHT_LOWER_LEG=null;
    this.RIGHT_FOOT=null;
    this.LEFT_UPPER_ARM=null;
    this.LEFT_LOWER_ARM=null;
    this.RIGHT_UPPER_ARM=null;
    this.RIGHT_LOWER_ARM=null;
  };
  
  var m_joints = {
    PELVIS_SPINE:null,
    SPINE_HEAD:null,
    LEFT_HIP:null,
    LEFT_KNEE:null,
    LEFT_ANKLE:null,
    RIGHT_HIP:null,
    RIGHT_KNEE:null,
    RIGHT_ANKLE:null,
    LEFT_SHOULDER:null,
    LEFT_ELBOW:null,
    RIGHT_SHOULDER:null,
    RIGHT_ELBOW:null
  };

  var CONSTRAINT_DEBUG_SIZE = 0.1;

  var m_ownerWorld = this.m_ownerWorld = da.getDynamicsWorld();
  var m_shapes = new BodyParts();
  var m_bodies = new BodyParts();

  // Setup the geometry
  m_shapes.PELVIS = new Ammo.btCapsuleShapeX((0.15), (0.15));
  m_shapes.SPINE = new Ammo.btCapsuleShape((0.15), (0.28));
  m_shapes.HEAD = new Ammo.btCapsuleShape((0.15), (0.05));
  m_shapes.LEFT_UPPER_LEG = new Ammo.btCapsuleShape((0.07), (0.45));
  m_shapes.LEFT_LOWER_LEG = new Ammo.btCapsuleShape((0.06), (0.37));
  m_shapes.LEFT_FOOT = new Ammo.btCapsuleShapeZ((0.06), (0.25));
  m_shapes.RIGHT_UPPER_LEG = new Ammo.btCapsuleShape((0.07), (0.45));
  m_shapes.RIGHT_LOWER_LEG = new Ammo.btCapsuleShape((0.06), (0.37));
  m_shapes.RIGHT_FOOT = new Ammo.btCapsuleShapeZ((0.06), (0.25));
  m_shapes.LEFT_UPPER_ARM = new Ammo.btCapsuleShape((0.05), (0.33));
  m_shapes.LEFT_LOWER_ARM = new Ammo.btCapsuleShape((0.04), (0.25));
  m_shapes.RIGHT_UPPER_ARM = new Ammo.btCapsuleShape((0.05), (0.33));
  m_shapes.RIGHT_LOWER_ARM = new Ammo.btCapsuleShape((0.04), (0.25));

  // Setup all the rigid bodies
  var offset = new Ammo.btTransform(); offset.setIdentity();
  offset.setOrigin(positionOffset);

  var transform = new Ammo.btTransform();
  transform.setIdentity();
  transform.setOrigin(new Ammo.btVector3((0.), (1.), (.0)));
  transform.op_mul(offset);
  m_bodies.PELVIS = da.localCreateRigidBody((1.),transform,m_shapes.PELVIS);
  
  transform.setIdentity();
  transform.setOrigin(new Ammo.btVector3((0.), (1.2), (0.)));
  transform.op_mul(offset)
  m_bodies.SPINE = da.localCreateRigidBody((1.), transform, m_shapes.SPINE);
  transform.setIdentity();
  transform.setOrigin(new Ammo.btVector3((0.), (1.6), (0.)));
  transform.op_mul(offset);
  m_bodies.HEAD = da.localCreateRigidBody((1.), transform, m_shapes.HEAD);

  transform.setIdentity();
  transform.setOrigin(new Ammo.btVector3((-0.18), (0.65), (0.)));
  transform.op_mul(offset);
  m_bodies.LEFT_UPPER_LEG = da.localCreateRigidBody((1.), transform, m_shapes.LEFT_UPPER_LEG);

  transform.setIdentity();
  transform.setOrigin(new Ammo.btVector3((-0.18), (0.2), (0.)));
  transform.op_mul(offset);
  m_bodies.LEFT_LOWER_LEG = da.localCreateRigidBody((1.), transform, m_shapes.LEFT_LOWER_LEG);

  transform.setIdentity();
  transform.setOrigin(new Ammo.btVector3((-0.18), (0.0), (-0.1)));
  transform.op_mul(offset);
  m_bodies.LEFT_FOOT = da.localCreateRigidBody((1.), transform, m_shapes.LEFT_FOOT);

  transform.setIdentity();
  transform.setOrigin(new Ammo.btVector3((0.18), (0.65), (0.)));
  transform.op_mul(offset);
  m_bodies.RIGHT_UPPER_LEG = da.localCreateRigidBody((1.), transform, m_shapes.RIGHT_UPPER_LEG);

  transform.setIdentity();
  transform.setOrigin(new Ammo.btVector3((0.18), (0.2), (0.)));
  transform.op_mul(offset);
  m_bodies.RIGHT_LOWER_LEG = da.localCreateRigidBody((1.), transform, m_shapes.RIGHT_LOWER_LEG);

  transform.setIdentity();
  transform.setOrigin(new Ammo.btVector3((0.18), (0.0), (-0.1)));
  transform.op_mul(offset);
  m_bodies.RIGHT_FOOT = da.localCreateRigidBody((1.), transform, m_shapes.RIGHT_FOOT);

  transform.setIdentity();
  transform.setOrigin(new Ammo.btVector3((-0.35), (1.45), (0.)));
  transform.getBasis().setEulerZYX(0,0,Math.PI/2);
  transform.op_mul(offset);
  m_bodies.LEFT_UPPER_ARM = da.localCreateRigidBody((1.), transform, m_shapes.LEFT_UPPER_ARM);

  transform.setIdentity();
  transform.setOrigin(new Ammo.btVector3((-0.7), (1.45), (0.)));
  transform.getBasis().setEulerZYX(0,0,Math.PI/2);
  transform.op_mul(offset);
  m_bodies.LEFT_LOWER_ARM = da.localCreateRigidBody((1.), transform, m_shapes.LEFT_LOWER_ARM);

  transform.setIdentity();
  transform.setOrigin(new Ammo.btVector3((0.35), (1.45), (0.)));
  transform.getBasis().setEulerZYX(0,0,-Math.PI/2);
  transform.op_mul(offset);
  m_bodies.RIGHT_UPPER_ARM = da.localCreateRigidBody((1.), transform, m_shapes.RIGHT_UPPER_ARM);

  transform.setIdentity();
  transform.setOrigin(new Ammo.btVector3((0.7), (1.45), (0.)));
  transform.getBasis().setEulerZYX(0,0,-Math.PI/2);
  transform.op_mul(offset);
  m_bodies.RIGHT_LOWER_ARM = da.localCreateRigidBody((1.), transform, m_shapes.RIGHT_LOWER_ARM);

  // Setup some damping on the m_bodies
  for(e in m_bodies){
    m_bodies[e].setDamping(0.05, 0.85);
    m_bodies[e].setDeactivationTime(0.8);
    m_bodies[e].setSleepingThresholds(1.6, 2.5);
  }

  // Now setup the constraints
  //var hingeC = new Ammo.btHingeConstraint();
  //var coneC = new Ammo.btConeTwistConstraint();

  var localA = new Ammo.btTransform();
  var localB = new Ammo.btTransform();
  /*
  localA.setIdentity(); localB.setIdentity();
  localA.getBasis().setEulerZYX(0,Math.PI/2,0); localA.setOrigin(new Ammo.btVector3((0.), (0.15), (0.)));
  localB.getBasis().setEulerZYX(0,Math.PI/2,0); localB.setOrigin(new Ammo.btVector3((0.), (-0.15), (0.)));
  var hingeC = new Ammo.btHingeConstraint(m_bodies.PELVIS, m_bodies.SPINE, localA, localB);
  hingeC.setLimit((-Math.PI/4), (Math.PI/2));
  */

  var hingeC = new Ammo.btHingeConstraint(m_bodies.PELVIS,
					   m_bodies.SPINE,
					   new Ammo.btVector3(0,0.15,-0.05),
					   new Ammo.btVector3(0,-0.15,0),
					   new Ammo.btVector3(0,0,0.15),
					   new Ammo.btVector3(0,0,0.15));
  m_joints.JOINT_PELVIS_SPINE = hingeC;
  hingeC.setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

  m_ownerWorld.addConstraint(m_joints.JOINT_PELVIS_SPINE, true);

  localA.setIdentity(); localB.setIdentity();
  localA.getBasis().setEulerZYX(0,0,Math.PI/2); localA.setOrigin(new Ammo.btVector3((0.), (0.30), (0.)));
  localB.getBasis().setEulerZYX(0,0,Math.PI/2); localB.setOrigin(new Ammo.btVector3((0.), (-0.14), (0.)));
  var coneC = new Ammo.btConeTwistConstraint(m_bodies.SPINE, m_bodies.HEAD, localA, localB);
  coneC.setLimit(Math.PI/4, Math.PI/4, Math.PI/2);
  m_joints.JOINT_SPINE_HEAD = coneC;
  coneC.setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

  m_ownerWorld.addConstraint(m_joints.JOINT_SPINE_HEAD, true);


  localA.setIdentity(); localB.setIdentity();
  localA.getBasis().setEulerZYX(0,0,-Math.PI/4*5); localA.setOrigin(new Ammo.btVector3((-0.18), (-0.10), (-0.03)));
  localB.getBasis().setEulerZYX(0,0,-Math.PI/4*5); localB.setOrigin(new Ammo.btVector3((0.), (0.225), (0.)));
  coneC = new Ammo.btConeTwistConstraint(m_bodies.PELVIS, m_bodies.LEFT_UPPER_LEG, localA, localB);
  coneC.setLimit(Math.PI/4, Math.PI/4, 0);
  m_joints.JOINT_LEFT_HIP = coneC;
  coneC.setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

  m_ownerWorld.addConstraint(m_joints.JOINT_LEFT_HIP, true);

  /*localA.setIdentity(); localB.setIdentity();
  localA.getBasis().setEulerZYX(0,Math.PI/2,0); localA.setOrigin(new Ammo.btVector3((0.), (-0.225), (0.)));
  localB.getBasis().setEulerZYX(0,Math.PI/2,0); localB.setOrigin(new Ammo.btVector3((0.), (0.185), (0.)));
  hingeC =  new Ammo.btHingeConstraint(m_bodies.LEFT_UPPER_LEG, m_bodies.LEFT_LOWER_LEG, localA, localB);
  hingeC.setLimit((0), (Math.PI/2));
  */
  hingeC =  new Ammo.btHingeConstraint(m_bodies.LEFT_UPPER_LEG,
				       m_bodies.LEFT_LOWER_LEG,
				       new Ammo.btVector3(0,-0.225,0),
				       new Ammo.btVector3(0,0.185,0),
				       new Ammo.btVector3(1,0,0),
				       new Ammo.btVector3(1,0,0));

  hingeC.setLimit((-0.1), (Math.PI*0.9));
  m_joints.JOINT_LEFT_KNEE = hingeC;
  hingeC.setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

  m_ownerWorld.addConstraint(m_joints.JOINT_LEFT_KNEE, true);

  hingeC =  new Ammo.btHingeConstraint(m_bodies.LEFT_LOWER_LEG,
				       m_bodies.LEFT_FOOT,
				       new Ammo.btVector3(0,-0.225,0),
				       new Ammo.btVector3(0,0.015,0.1),
				       new Ammo.btVector3(1,0,0),
				       new Ammo.btVector3(1,0,0));

  hingeC.setLimit((-1), (1));
  m_joints.JOINT_LEFT_ANKLE = hingeC;
  hingeC.setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

  m_ownerWorld.addConstraint(m_joints.JOINT_LEFT_ANKLE, true);

  localA.setIdentity(); localB.setIdentity();
  localA.getBasis().setEulerZYX(0,0,Math.PI/4); localA.setOrigin(new Ammo.btVector3((0.18), (-0.10), (-0.03)));
  localB.getBasis().setEulerZYX(0,0,Math.PI/4); localB.setOrigin(new Ammo.btVector3((0.), (0.225), (0.)));
  coneC = new Ammo.btConeTwistConstraint(m_bodies.PELVIS, m_bodies.RIGHT_UPPER_LEG, localA, localB);
  coneC.setLimit(Math.PI/4, Math.PI/4, 0);
  m_joints.JOINT_RIGHT_HIP = coneC;
  coneC.setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

  m_ownerWorld.addConstraint(m_joints.JOINT_RIGHT_HIP, true);

  /*
    localA.setIdentity(); localB.setIdentity();
  localA.getBasis().setEulerZYX(0,Math.PI/2,0); localA.setOrigin(new Ammo.btVector3((0.), (-0.225), (0.)));
  localB.getBasis().setEulerZYX(0,Math.PI/2,0); localB.setOrigin(new Ammo.btVector3((0.), (0.185), (0.)));
  hingeC =  new Ammo.btHingeConstraint(m_bodies.RIGHT_UPPER_LEG, m_bodies.RIGHT_LOWER_LEG, localA, localB);
  hingeC.setLimit((0), (Math.PI/2));
  */
  hingeC =  new Ammo.btHingeConstraint(m_bodies.RIGHT_UPPER_LEG,
				       m_bodies.RIGHT_LOWER_LEG,
				       new Ammo.btVector3(0,-0.225,0),
				       new Ammo.btVector3(0,0.185,0),
				       new Ammo.btVector3(1,0,0),
				       new Ammo.btVector3(1,0,0));

  hingeC.setLimit((-0.1), (Math.PI*0.9));
  m_joints.JOINT_RIGHT_KNEE = hingeC;
  hingeC.setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

  m_ownerWorld.addConstraint(m_joints.JOINT_RIGHT_KNEE, true);

  hingeC =  new Ammo.btHingeConstraint(m_bodies.RIGHT_LOWER_LEG,
				       m_bodies.RIGHT_FOOT,
				       new Ammo.btVector3(0,-0.225,0),
				       new Ammo.btVector3(0,0.015,0.1),
				       new Ammo.btVector3(1,0,0),
				       new Ammo.btVector3(1,0,0));

  hingeC.setLimit((-1), (1));
  m_joints.JOINT_RIGHT_ANKLE = hingeC;
  hingeC.setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

  m_ownerWorld.addConstraint(m_joints.JOINT_RIGHT_ANKLE, true);


  localA.setIdentity(); localB.setIdentity();
  localA.getBasis().setEulerZYX(0,0,Math.PI); localA.setOrigin(new Ammo.btVector3((-0.2), (0.15), (0.)));
  localB.getBasis().setEulerZYX(0,0,Math.PI/2); localB.setOrigin(new Ammo.btVector3((0.), (-0.18), (0.)));
  coneC = new Ammo.btConeTwistConstraint(m_bodies.SPINE, m_bodies.LEFT_UPPER_ARM, localA, localB);
  coneC.setLimit(Math.PI/2, Math.PI/2, 0);
  coneC.setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

  m_joints.JOINT_LEFT_SHOULDER = coneC;
  m_ownerWorld.addConstraint(m_joints.JOINT_LEFT_SHOULDER, true);

  /*
    localA.setIdentity(); localB.setIdentity();
  localA.getBasis().setEulerZYX(0,Math.PI/2,0); localA.setOrigin(new Ammo.btVector3((0.), (0.18), (0.)));
  localB.getBasis().setEulerZYX(0,Math.PI/2,0); localB.setOrigin(new Ammo.btVector3((0.), (-0.14), (0.)));
  hingeC =  new Ammo.btHingeConstraint(m_bodies.LEFT_UPPER_ARM,
				       m_bodies.LEFT_LOWER_ARM,
				       localA, localB);
  //		hingeC.setLimit((-Math.PI/2), (0));
  hingeC.setLimit((0), (Math.PI/2));
  */
  hingeC =  new Ammo.btHingeConstraint(m_bodies.LEFT_UPPER_ARM,
				       m_bodies.LEFT_LOWER_ARM,
				       new Ammo.btVector3(0,0.18,0),
				       new Ammo.btVector3(0,-0.14,0),
				       new Ammo.btVector3(0,0,0.18),
				       new Ammo.btVector3(0,0,0.14));
  //		hingeC.setLimit((-Math.PI/2), (0));
  hingeC.setLimit(-Math.PI/2,0);
  m_joints.JOINT_LEFT_ELBOW = hingeC;
  hingeC.setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

  m_ownerWorld.addConstraint(m_joints.JOINT_LEFT_ELBOW, true);

  localA.setIdentity(); localB.setIdentity();
  localA.getBasis().setEulerZYX(0,0,0); localA.setOrigin(new Ammo.btVector3((0.2), (0.15), (0.)));
  localB.getBasis().setEulerZYX(0,0,Math.PI/2); localB.setOrigin(new Ammo.btVector3((0.), (-0.18), (0.)));
  coneC = new Ammo.btConeTwistConstraint(m_bodies.SPINE, m_bodies.RIGHT_UPPER_ARM, localA, localB);
  coneC.setLimit(Math.PI/2, Math.PI/2, 0);
  m_joints.JOINT_RIGHT_SHOULDER = coneC;
  coneC.setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

  m_ownerWorld.addConstraint(m_joints.JOINT_RIGHT_SHOULDER, true);

  /*
  var hingeC1_localA = new Ammo.btTransform();
  hingeC1_localA.setIdentity();
  var b1 = hingeC1_localA.getBasis();
  b1.setEulerZYX(0,Math.PI/2,0);
  hingeC1_localA.setBasis(b1);
  hingeC1_localA.setOrigin(new Ammo.btVector3(0., 0.18, 0.));
  var hingeC1_localB = new Ammo.btTransform();
  hingeC1_localB.setIdentity();
  var b2 = hingeC1_localB.getBasis();
  b2.setEulerZYX(0,Math.PI/2,0);
  hingeC1_localB.setBasis(b2);
  hingeC1_localB.setOrigin(new Ammo.btVector3(0., -0.14, 0.));
  var hingeC1 = new Ammo.btHingeConstraint(m_bodies.RIGHT_UPPER_ARM,
					   m_bodies.RIGHT_LOWER_ARM,			   
					   hingeC1_localA,
					   hingeC1_localB);
  */
  /*hingeC1.setFrames(hingeC1_localA,
    hingeC1_localB);*/

  var hingeC1 = new Ammo.btHingeConstraint(m_bodies.RIGHT_UPPER_ARM,
					   m_bodies.RIGHT_LOWER_ARM,
					   new Ammo.btVector3(0,0.18,0),
					   new Ammo.btVector3(0,-0.14,0),
					   new Ammo.btVector3(0,0,0.18),
					   new Ammo.btVector3(0,0,0.14));
  hingeC1.setLimit(0.0, Math.PI/2);
  //hingeC1.setLimit((-Math.PI/2), (0.0));
  //hingeC.setLimit((0), (Math.PI/2));
  m_joints.JOINT_RIGHT_ELBOW = hingeC1;
  hingeC1.setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

  m_ownerWorld.addConstraint(m_joints.JOINT_RIGHT_ELBOW, true);
}
