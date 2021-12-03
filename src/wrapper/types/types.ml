(* THIS FILE IS GENERATED AUTOMATICALLY, DO NOT EDIT BY HAND *)

open Ctypes

module Bindings (S : Cstubs.Types.TYPE) = struct
  open S

  type mjtByte = Unsigned.UChar.t

  let mjtByte = uchar

  type mjtNum = float

  let mjtNum = double
  let mjfItemEnable = static_funptr (int @-> ptr void @-> returning int)

  (* -------------------------------------------------------------------------------- *)
  (* ------------------------------ mjmodel.h --------------------------------------- *)
  (* -------------------------------------------------------------------------------- *)

  (**  disable default feature bitflags *)
  type mjtDisableBit =
    | MjDSBL_CONSTRAINT
    | MjDSBL_EQUALITY
    | MjDSBL_FRICTIONLOSS
    | MjDSBL_LIMIT
    | MjDSBL_CONTACT
    | MjDSBL_PASSIVE
    | MjDSBL_GRAVITY
    | MjDSBL_CLAMPCTRL
    | MjDSBL_WARMSTART
    | MjDSBL_FILTERPARENT
    | MjDSBL_ACTUATION
    | MjDSBL_REFSAFE
    | MjNDISABLE

  let mjDSBL_CONSTRAINT = constant "mjDSBL_CONSTRAINT" int64_t
  let mjDSBL_EQUALITY = constant "mjDSBL_EQUALITY" int64_t
  let mjDSBL_FRICTIONLOSS = constant "mjDSBL_FRICTIONLOSS" int64_t
  let mjDSBL_LIMIT = constant "mjDSBL_LIMIT" int64_t
  let mjDSBL_CONTACT = constant "mjDSBL_CONTACT" int64_t
  let mjDSBL_PASSIVE = constant "mjDSBL_PASSIVE" int64_t
  let mjDSBL_GRAVITY = constant "mjDSBL_GRAVITY" int64_t
  let mjDSBL_CLAMPCTRL = constant "mjDSBL_CLAMPCTRL" int64_t
  let mjDSBL_WARMSTART = constant "mjDSBL_WARMSTART" int64_t
  let mjDSBL_FILTERPARENT = constant "mjDSBL_FILTERPARENT" int64_t
  let mjDSBL_ACTUATION = constant "mjDSBL_ACTUATION" int64_t
  let mjDSBL_REFSAFE = constant "mjDSBL_REFSAFE" int64_t
  let mjNDISABLE = constant "mjNDISABLE" int64_t

  let mjtDisableBit =
    S.enum
      "mjtDisableBit"
      ~typedef:true
      [ MjDSBL_CONSTRAINT, mjDSBL_CONSTRAINT
      ; MjDSBL_EQUALITY, mjDSBL_EQUALITY
      ; MjDSBL_FRICTIONLOSS, mjDSBL_FRICTIONLOSS
      ; MjDSBL_LIMIT, mjDSBL_LIMIT
      ; MjDSBL_CONTACT, mjDSBL_CONTACT
      ; MjDSBL_PASSIVE, mjDSBL_PASSIVE
      ; MjDSBL_GRAVITY, mjDSBL_GRAVITY
      ; MjDSBL_CLAMPCTRL, mjDSBL_CLAMPCTRL
      ; MjDSBL_WARMSTART, mjDSBL_WARMSTART
      ; MjDSBL_FILTERPARENT, mjDSBL_FILTERPARENT
      ; MjDSBL_ACTUATION, mjDSBL_ACTUATION
      ; MjDSBL_REFSAFE, mjDSBL_REFSAFE
      ; MjNDISABLE, mjNDISABLE
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtDisableBit element data type enum")


  (**  enable optional feature bitflags *)
  type mjtEnableBit =
    | MjENBL_OVERRIDE
    | MjENBL_ENERGY
    | MjENBL_FWDINV
    | MjENBL_SENSORNOISE
    | MjNENABLE

  let mjENBL_OVERRIDE = constant "mjENBL_OVERRIDE" int64_t
  let mjENBL_ENERGY = constant "mjENBL_ENERGY" int64_t
  let mjENBL_FWDINV = constant "mjENBL_FWDINV" int64_t
  let mjENBL_SENSORNOISE = constant "mjENBL_SENSORNOISE" int64_t
  let mjNENABLE = constant "mjNENABLE" int64_t

  let mjtEnableBit =
    S.enum
      "mjtEnableBit"
      ~typedef:true
      [ MjENBL_OVERRIDE, mjENBL_OVERRIDE
      ; MjENBL_ENERGY, mjENBL_ENERGY
      ; MjENBL_FWDINV, mjENBL_FWDINV
      ; MjENBL_SENSORNOISE, mjENBL_SENSORNOISE
      ; MjNENABLE, mjNENABLE
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtEnableBit element data type enum")


  (**  type of degree of freedom *)
  type mjtJoint =
    | MjJNT_FREE
    | MjJNT_BALL
    | MjJNT_SLIDE
    | MjJNT_HINGE

  let mjJNT_FREE = constant "mjJNT_FREE" int64_t
  let mjJNT_BALL = constant "mjJNT_BALL" int64_t
  let mjJNT_SLIDE = constant "mjJNT_SLIDE" int64_t
  let mjJNT_HINGE = constant "mjJNT_HINGE" int64_t

  let mjtJoint =
    S.enum
      "mjtJoint"
      ~typedef:true
      [ MjJNT_FREE, mjJNT_FREE
      ; MjJNT_BALL, mjJNT_BALL
      ; MjJNT_SLIDE, mjJNT_SLIDE
      ; MjJNT_HINGE, mjJNT_HINGE
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtJoint element data type enum")


  (**  type of geometric shape *)
  type mjtGeom =
    | MjGEOM_PLANE
    | MjGEOM_HFIELD
    | MjGEOM_SPHERE
    | MjGEOM_CAPSULE
    | MjGEOM_ELLIPSOID
    | MjGEOM_CYLINDER
    | MjGEOM_BOX
    | MjGEOM_MESH
    | MjNGEOMTYPES
    | MjGEOM_ARROW
    | MjGEOM_ARROW1
    | MjGEOM_ARROW2
    | MjGEOM_LINE
    | MjGEOM_SKIN
    | MjGEOM_LABEL
    | MjGEOM_NONE

  let mjGEOM_PLANE = constant "mjGEOM_PLANE" int64_t
  let mjGEOM_HFIELD = constant "mjGEOM_HFIELD" int64_t
  let mjGEOM_SPHERE = constant "mjGEOM_SPHERE" int64_t
  let mjGEOM_CAPSULE = constant "mjGEOM_CAPSULE" int64_t
  let mjGEOM_ELLIPSOID = constant "mjGEOM_ELLIPSOID" int64_t
  let mjGEOM_CYLINDER = constant "mjGEOM_CYLINDER" int64_t
  let mjGEOM_BOX = constant "mjGEOM_BOX" int64_t
  let mjGEOM_MESH = constant "mjGEOM_MESH" int64_t
  let mjNGEOMTYPES = constant "mjNGEOMTYPES" int64_t
  let mjGEOM_ARROW = constant "mjGEOM_ARROW" int64_t
  let mjGEOM_ARROW1 = constant "mjGEOM_ARROW1" int64_t
  let mjGEOM_ARROW2 = constant "mjGEOM_ARROW2" int64_t
  let mjGEOM_LINE = constant "mjGEOM_LINE" int64_t
  let mjGEOM_SKIN = constant "mjGEOM_SKIN" int64_t
  let mjGEOM_LABEL = constant "mjGEOM_LABEL" int64_t
  let mjGEOM_NONE = constant "mjGEOM_NONE" int64_t

  let mjtGeom =
    S.enum
      "mjtGeom"
      ~typedef:true
      [ MjGEOM_PLANE, mjGEOM_PLANE
      ; MjGEOM_HFIELD, mjGEOM_HFIELD
      ; MjGEOM_SPHERE, mjGEOM_SPHERE
      ; MjGEOM_CAPSULE, mjGEOM_CAPSULE
      ; MjGEOM_ELLIPSOID, mjGEOM_ELLIPSOID
      ; MjGEOM_CYLINDER, mjGEOM_CYLINDER
      ; MjGEOM_BOX, mjGEOM_BOX
      ; MjGEOM_MESH, mjGEOM_MESH
      ; MjNGEOMTYPES, mjNGEOMTYPES
      ; MjGEOM_ARROW, mjGEOM_ARROW
      ; MjGEOM_ARROW1, mjGEOM_ARROW1
      ; MjGEOM_ARROW2, mjGEOM_ARROW2
      ; MjGEOM_LINE, mjGEOM_LINE
      ; MjGEOM_SKIN, mjGEOM_SKIN
      ; MjGEOM_LABEL, mjGEOM_LABEL
      ; MjGEOM_NONE, mjGEOM_NONE
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtGeom element data type enum")


  (**  tracking mode for camera and light *)
  type mjtCamLight =
    | MjCAMLIGHT_FIXED
    | MjCAMLIGHT_TRACK
    | MjCAMLIGHT_TRACKCOM
    | MjCAMLIGHT_TARGETBODY
    | MjCAMLIGHT_TARGETBODYCOM

  let mjCAMLIGHT_FIXED = constant "mjCAMLIGHT_FIXED" int64_t
  let mjCAMLIGHT_TRACK = constant "mjCAMLIGHT_TRACK" int64_t
  let mjCAMLIGHT_TRACKCOM = constant "mjCAMLIGHT_TRACKCOM" int64_t
  let mjCAMLIGHT_TARGETBODY = constant "mjCAMLIGHT_TARGETBODY" int64_t
  let mjCAMLIGHT_TARGETBODYCOM = constant "mjCAMLIGHT_TARGETBODYCOM" int64_t

  let mjtCamLight =
    S.enum
      "mjtCamLight"
      ~typedef:true
      [ MjCAMLIGHT_FIXED, mjCAMLIGHT_FIXED
      ; MjCAMLIGHT_TRACK, mjCAMLIGHT_TRACK
      ; MjCAMLIGHT_TRACKCOM, mjCAMLIGHT_TRACKCOM
      ; MjCAMLIGHT_TARGETBODY, mjCAMLIGHT_TARGETBODY
      ; MjCAMLIGHT_TARGETBODYCOM, mjCAMLIGHT_TARGETBODYCOM
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtCamLight element data type enum")


  (**  type of texture *)
  type mjtTexture =
    | MjTEXTURE_2D
    | MjTEXTURE_CUBE
    | MjTEXTURE_SKYBOX

  let mjTEXTURE_2D = constant "mjTEXTURE_2D" int64_t
  let mjTEXTURE_CUBE = constant "mjTEXTURE_CUBE" int64_t
  let mjTEXTURE_SKYBOX = constant "mjTEXTURE_SKYBOX" int64_t

  let mjtTexture =
    S.enum
      "mjtTexture"
      ~typedef:true
      [ MjTEXTURE_2D, mjTEXTURE_2D
      ; MjTEXTURE_CUBE, mjTEXTURE_CUBE
      ; MjTEXTURE_SKYBOX, mjTEXTURE_SKYBOX
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtTexture element data type enum")


  (**  integrator mode *)
  type mjtIntegrator =
    | MjINT_EULER
    | MjINT_RK4

  let mjINT_EULER = constant "mjINT_EULER" int64_t
  let mjINT_RK4 = constant "mjINT_RK4" int64_t

  let mjtIntegrator =
    S.enum
      "mjtIntegrator"
      ~typedef:true
      [ MjINT_EULER, mjINT_EULER; MjINT_RK4, mjINT_RK4 ]
      ~unexpected:(fun _ -> failwith "unexpected mjtIntegrator element data type enum")


  (**  collision mode for selecting geom pairs *)
  type mjtCollision =
    | MjCOL_ALL
    | MjCOL_PAIR
    | MjCOL_DYNAMIC

  let mjCOL_ALL = constant "mjCOL_ALL" int64_t
  let mjCOL_PAIR = constant "mjCOL_PAIR" int64_t
  let mjCOL_DYNAMIC = constant "mjCOL_DYNAMIC" int64_t

  let mjtCollision =
    S.enum
      "mjtCollision"
      ~typedef:true
      [ MjCOL_ALL, mjCOL_ALL; MjCOL_PAIR, mjCOL_PAIR; MjCOL_DYNAMIC, mjCOL_DYNAMIC ]
      ~unexpected:(fun _ -> failwith "unexpected mjtCollision element data type enum")


  (**  type of friction cone *)
  type mjtCone =
    | MjCONE_PYRAMIDAL
    | MjCONE_ELLIPTIC

  let mjCONE_PYRAMIDAL = constant "mjCONE_PYRAMIDAL" int64_t
  let mjCONE_ELLIPTIC = constant "mjCONE_ELLIPTIC" int64_t

  let mjtCone =
    S.enum
      "mjtCone"
      ~typedef:true
      [ MjCONE_PYRAMIDAL, mjCONE_PYRAMIDAL; MjCONE_ELLIPTIC, mjCONE_ELLIPTIC ]
      ~unexpected:(fun _ -> failwith "unexpected mjtCone element data type enum")


  (**  type of constraint Jacobian *)
  type mjtJacobian =
    | MjJAC_DENSE
    | MjJAC_SPARSE
    | MjJAC_AUTO

  let mjJAC_DENSE = constant "mjJAC_DENSE" int64_t
  let mjJAC_SPARSE = constant "mjJAC_SPARSE" int64_t
  let mjJAC_AUTO = constant "mjJAC_AUTO" int64_t

  let mjtJacobian =
    S.enum
      "mjtJacobian"
      ~typedef:true
      [ MjJAC_DENSE, mjJAC_DENSE; MjJAC_SPARSE, mjJAC_SPARSE; MjJAC_AUTO, mjJAC_AUTO ]
      ~unexpected:(fun _ -> failwith "unexpected mjtJacobian element data type enum")


  (**  constraint solver algorithm *)
  type mjtSolver =
    | MjSOL_PGS
    | MjSOL_CG
    | MjSOL_NEWTON

  let mjSOL_PGS = constant "mjSOL_PGS" int64_t
  let mjSOL_CG = constant "mjSOL_CG" int64_t
  let mjSOL_NEWTON = constant "mjSOL_NEWTON" int64_t

  let mjtSolver =
    S.enum
      "mjtSolver"
      ~typedef:true
      [ MjSOL_PGS, mjSOL_PGS; MjSOL_CG, mjSOL_CG; MjSOL_NEWTON, mjSOL_NEWTON ]
      ~unexpected:(fun _ -> failwith "unexpected mjtSolver element data type enum")


  (**  type of equality constraint *)
  type mjtEq =
    | MjEQ_CONNECT
    | MjEQ_WELD
    | MjEQ_JOINT
    | MjEQ_TENDON
    | MjEQ_DISTANCE

  let mjEQ_CONNECT = constant "mjEQ_CONNECT" int64_t
  let mjEQ_WELD = constant "mjEQ_WELD" int64_t
  let mjEQ_JOINT = constant "mjEQ_JOINT" int64_t
  let mjEQ_TENDON = constant "mjEQ_TENDON" int64_t
  let mjEQ_DISTANCE = constant "mjEQ_DISTANCE" int64_t

  let mjtEq =
    S.enum
      "mjtEq"
      ~typedef:true
      [ MjEQ_CONNECT, mjEQ_CONNECT
      ; MjEQ_WELD, mjEQ_WELD
      ; MjEQ_JOINT, mjEQ_JOINT
      ; MjEQ_TENDON, mjEQ_TENDON
      ; MjEQ_DISTANCE, mjEQ_DISTANCE
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtEq element data type enum")


  (**  type of tendon wrap object *)
  type mjtWrap =
    | MjWRAP_NONE
    | MjWRAP_JOINT
    | MjWRAP_PULLEY
    | MjWRAP_SITE
    | MjWRAP_SPHERE
    | MjWRAP_CYLINDER

  let mjWRAP_NONE = constant "mjWRAP_NONE" int64_t
  let mjWRAP_JOINT = constant "mjWRAP_JOINT" int64_t
  let mjWRAP_PULLEY = constant "mjWRAP_PULLEY" int64_t
  let mjWRAP_SITE = constant "mjWRAP_SITE" int64_t
  let mjWRAP_SPHERE = constant "mjWRAP_SPHERE" int64_t
  let mjWRAP_CYLINDER = constant "mjWRAP_CYLINDER" int64_t

  let mjtWrap =
    S.enum
      "mjtWrap"
      ~typedef:true
      [ MjWRAP_NONE, mjWRAP_NONE
      ; MjWRAP_JOINT, mjWRAP_JOINT
      ; MjWRAP_PULLEY, mjWRAP_PULLEY
      ; MjWRAP_SITE, mjWRAP_SITE
      ; MjWRAP_SPHERE, mjWRAP_SPHERE
      ; MjWRAP_CYLINDER, mjWRAP_CYLINDER
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtWrap element data type enum")


  (**  type of actuator transmission *)
  type mjtTrn =
    | MjTRN_JOINT
    | MjTRN_JOINTINPARENT
    | MjTRN_SLIDERCRANK
    | MjTRN_TENDON
    | MjTRN_SITE
    | MjTRN_UNDEFINED

  let mjTRN_JOINT = constant "mjTRN_JOINT" int64_t
  let mjTRN_JOINTINPARENT = constant "mjTRN_JOINTINPARENT" int64_t
  let mjTRN_SLIDERCRANK = constant "mjTRN_SLIDERCRANK" int64_t
  let mjTRN_TENDON = constant "mjTRN_TENDON" int64_t
  let mjTRN_SITE = constant "mjTRN_SITE" int64_t
  let mjTRN_UNDEFINED = constant "mjTRN_UNDEFINED" int64_t

  let mjtTrn =
    S.enum
      "mjtTrn"
      ~typedef:true
      [ MjTRN_JOINT, mjTRN_JOINT
      ; MjTRN_JOINTINPARENT, mjTRN_JOINTINPARENT
      ; MjTRN_SLIDERCRANK, mjTRN_SLIDERCRANK
      ; MjTRN_TENDON, mjTRN_TENDON
      ; MjTRN_SITE, mjTRN_SITE
      ; MjTRN_UNDEFINED, mjTRN_UNDEFINED
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtTrn element data type enum")


  (**  type of actuator dynamics *)
  type mjtDyn =
    | MjDYN_NONE
    | MjDYN_INTEGRATOR
    | MjDYN_FILTER
    | MjDYN_MUSCLE
    | MjDYN_USER

  let mjDYN_NONE = constant "mjDYN_NONE" int64_t
  let mjDYN_INTEGRATOR = constant "mjDYN_INTEGRATOR" int64_t
  let mjDYN_FILTER = constant "mjDYN_FILTER" int64_t
  let mjDYN_MUSCLE = constant "mjDYN_MUSCLE" int64_t
  let mjDYN_USER = constant "mjDYN_USER" int64_t

  let mjtDyn =
    S.enum
      "mjtDyn"
      ~typedef:true
      [ MjDYN_NONE, mjDYN_NONE
      ; MjDYN_INTEGRATOR, mjDYN_INTEGRATOR
      ; MjDYN_FILTER, mjDYN_FILTER
      ; MjDYN_MUSCLE, mjDYN_MUSCLE
      ; MjDYN_USER, mjDYN_USER
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtDyn element data type enum")


  (**  type of actuator gain *)
  type mjtGain =
    | MjGAIN_FIXED
    | MjGAIN_MUSCLE
    | MjGAIN_USER

  let mjGAIN_FIXED = constant "mjGAIN_FIXED" int64_t
  let mjGAIN_MUSCLE = constant "mjGAIN_MUSCLE" int64_t
  let mjGAIN_USER = constant "mjGAIN_USER" int64_t

  let mjtGain =
    S.enum
      "mjtGain"
      ~typedef:true
      [ MjGAIN_FIXED, mjGAIN_FIXED
      ; MjGAIN_MUSCLE, mjGAIN_MUSCLE
      ; MjGAIN_USER, mjGAIN_USER
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtGain element data type enum")


  (**  type of actuator bias *)
  type mjtBias =
    | MjBIAS_NONE
    | MjBIAS_AFFINE
    | MjBIAS_MUSCLE
    | MjBIAS_USER

  let mjBIAS_NONE = constant "mjBIAS_NONE" int64_t
  let mjBIAS_AFFINE = constant "mjBIAS_AFFINE" int64_t
  let mjBIAS_MUSCLE = constant "mjBIAS_MUSCLE" int64_t
  let mjBIAS_USER = constant "mjBIAS_USER" int64_t

  let mjtBias =
    S.enum
      "mjtBias"
      ~typedef:true
      [ MjBIAS_NONE, mjBIAS_NONE
      ; MjBIAS_AFFINE, mjBIAS_AFFINE
      ; MjBIAS_MUSCLE, mjBIAS_MUSCLE
      ; MjBIAS_USER, mjBIAS_USER
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtBias element data type enum")


  (**  type of MujoCo object *)
  type mjtObj =
    | MjOBJ_UNKNOWN
    | MjOBJ_BODY
    | MjOBJ_XBODY
    | MjOBJ_JOINT
    | MjOBJ_DOF
    | MjOBJ_GEOM
    | MjOBJ_SITE
    | MjOBJ_CAMERA
    | MjOBJ_LIGHT
    | MjOBJ_MESH
    | MjOBJ_SKIN
    | MjOBJ_HFIELD
    | MjOBJ_TEXTURE
    | MjOBJ_MATERIAL
    | MjOBJ_PAIR
    | MjOBJ_EXCLUDE
    | MjOBJ_EQUALITY
    | MjOBJ_TENDON
    | MjOBJ_ACTUATOR
    | MjOBJ_SENSOR
    | MjOBJ_NUMERIC
    | MjOBJ_TEXT
    | MjOBJ_TUPLE
    | MjOBJ_KEY

  let mjOBJ_UNKNOWN = constant "mjOBJ_UNKNOWN" int64_t
  let mjOBJ_BODY = constant "mjOBJ_BODY" int64_t
  let mjOBJ_XBODY = constant "mjOBJ_XBODY" int64_t
  let mjOBJ_JOINT = constant "mjOBJ_JOINT" int64_t
  let mjOBJ_DOF = constant "mjOBJ_DOF" int64_t
  let mjOBJ_GEOM = constant "mjOBJ_GEOM" int64_t
  let mjOBJ_SITE = constant "mjOBJ_SITE" int64_t
  let mjOBJ_CAMERA = constant "mjOBJ_CAMERA" int64_t
  let mjOBJ_LIGHT = constant "mjOBJ_LIGHT" int64_t
  let mjOBJ_MESH = constant "mjOBJ_MESH" int64_t
  let mjOBJ_SKIN = constant "mjOBJ_SKIN" int64_t
  let mjOBJ_HFIELD = constant "mjOBJ_HFIELD" int64_t
  let mjOBJ_TEXTURE = constant "mjOBJ_TEXTURE" int64_t
  let mjOBJ_MATERIAL = constant "mjOBJ_MATERIAL" int64_t
  let mjOBJ_PAIR = constant "mjOBJ_PAIR" int64_t
  let mjOBJ_EXCLUDE = constant "mjOBJ_EXCLUDE" int64_t
  let mjOBJ_EQUALITY = constant "mjOBJ_EQUALITY" int64_t
  let mjOBJ_TENDON = constant "mjOBJ_TENDON" int64_t
  let mjOBJ_ACTUATOR = constant "mjOBJ_ACTUATOR" int64_t
  let mjOBJ_SENSOR = constant "mjOBJ_SENSOR" int64_t
  let mjOBJ_NUMERIC = constant "mjOBJ_NUMERIC" int64_t
  let mjOBJ_TEXT = constant "mjOBJ_TEXT" int64_t
  let mjOBJ_TUPLE = constant "mjOBJ_TUPLE" int64_t
  let mjOBJ_KEY = constant "mjOBJ_KEY" int64_t

  let mjtObj =
    S.enum
      "mjtObj"
      ~typedef:true
      [ MjOBJ_UNKNOWN, mjOBJ_UNKNOWN
      ; MjOBJ_BODY, mjOBJ_BODY
      ; MjOBJ_XBODY, mjOBJ_XBODY
      ; MjOBJ_JOINT, mjOBJ_JOINT
      ; MjOBJ_DOF, mjOBJ_DOF
      ; MjOBJ_GEOM, mjOBJ_GEOM
      ; MjOBJ_SITE, mjOBJ_SITE
      ; MjOBJ_CAMERA, mjOBJ_CAMERA
      ; MjOBJ_LIGHT, mjOBJ_LIGHT
      ; MjOBJ_MESH, mjOBJ_MESH
      ; MjOBJ_SKIN, mjOBJ_SKIN
      ; MjOBJ_HFIELD, mjOBJ_HFIELD
      ; MjOBJ_TEXTURE, mjOBJ_TEXTURE
      ; MjOBJ_MATERIAL, mjOBJ_MATERIAL
      ; MjOBJ_PAIR, mjOBJ_PAIR
      ; MjOBJ_EXCLUDE, mjOBJ_EXCLUDE
      ; MjOBJ_EQUALITY, mjOBJ_EQUALITY
      ; MjOBJ_TENDON, mjOBJ_TENDON
      ; MjOBJ_ACTUATOR, mjOBJ_ACTUATOR
      ; MjOBJ_SENSOR, mjOBJ_SENSOR
      ; MjOBJ_NUMERIC, mjOBJ_NUMERIC
      ; MjOBJ_TEXT, mjOBJ_TEXT
      ; MjOBJ_TUPLE, mjOBJ_TUPLE
      ; MjOBJ_KEY, mjOBJ_KEY
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtObj element data type enum")


  (**  type of constraint *)
  type mjtConstraint =
    | MjCNSTR_EQUALITY
    | MjCNSTR_FRICTION_DOF
    | MjCNSTR_FRICTION_TENDON
    | MjCNSTR_LIMIT_JOINT
    | MjCNSTR_LIMIT_TENDON
    | MjCNSTR_CONTACT_FRICTIONLESS
    | MjCNSTR_CONTACT_PYRAMIDAL
    | MjCNSTR_CONTACT_ELLIPTIC

  let mjCNSTR_EQUALITY = constant "mjCNSTR_EQUALITY" int64_t
  let mjCNSTR_FRICTION_DOF = constant "mjCNSTR_FRICTION_DOF" int64_t
  let mjCNSTR_FRICTION_TENDON = constant "mjCNSTR_FRICTION_TENDON" int64_t
  let mjCNSTR_LIMIT_JOINT = constant "mjCNSTR_LIMIT_JOINT" int64_t
  let mjCNSTR_LIMIT_TENDON = constant "mjCNSTR_LIMIT_TENDON" int64_t
  let mjCNSTR_CONTACT_FRICTIONLESS = constant "mjCNSTR_CONTACT_FRICTIONLESS" int64_t
  let mjCNSTR_CONTACT_PYRAMIDAL = constant "mjCNSTR_CONTACT_PYRAMIDAL" int64_t
  let mjCNSTR_CONTACT_ELLIPTIC = constant "mjCNSTR_CONTACT_ELLIPTIC" int64_t

  let mjtConstraint =
    S.enum
      "mjtConstraint"
      ~typedef:true
      [ MjCNSTR_EQUALITY, mjCNSTR_EQUALITY
      ; MjCNSTR_FRICTION_DOF, mjCNSTR_FRICTION_DOF
      ; MjCNSTR_FRICTION_TENDON, mjCNSTR_FRICTION_TENDON
      ; MjCNSTR_LIMIT_JOINT, mjCNSTR_LIMIT_JOINT
      ; MjCNSTR_LIMIT_TENDON, mjCNSTR_LIMIT_TENDON
      ; MjCNSTR_CONTACT_FRICTIONLESS, mjCNSTR_CONTACT_FRICTIONLESS
      ; MjCNSTR_CONTACT_PYRAMIDAL, mjCNSTR_CONTACT_PYRAMIDAL
      ; MjCNSTR_CONTACT_ELLIPTIC, mjCNSTR_CONTACT_ELLIPTIC
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtConstraint element data type enum")


  (**  constraint state *)
  type mjtConstraintState =
    | MjCNSTRSTATE_SATISFIED
    | MjCNSTRSTATE_QUADRATIC
    | MjCNSTRSTATE_LINEARNEG
    | MjCNSTRSTATE_LINEARPOS
    | MjCNSTRSTATE_CONE

  let mjCNSTRSTATE_SATISFIED = constant "mjCNSTRSTATE_SATISFIED" int64_t
  let mjCNSTRSTATE_QUADRATIC = constant "mjCNSTRSTATE_QUADRATIC" int64_t
  let mjCNSTRSTATE_LINEARNEG = constant "mjCNSTRSTATE_LINEARNEG" int64_t
  let mjCNSTRSTATE_LINEARPOS = constant "mjCNSTRSTATE_LINEARPOS" int64_t
  let mjCNSTRSTATE_CONE = constant "mjCNSTRSTATE_CONE" int64_t

  let mjtConstraintState =
    S.enum
      "mjtConstraintState"
      ~typedef:true
      [ MjCNSTRSTATE_SATISFIED, mjCNSTRSTATE_SATISFIED
      ; MjCNSTRSTATE_QUADRATIC, mjCNSTRSTATE_QUADRATIC
      ; MjCNSTRSTATE_LINEARNEG, mjCNSTRSTATE_LINEARNEG
      ; MjCNSTRSTATE_LINEARPOS, mjCNSTRSTATE_LINEARPOS
      ; MjCNSTRSTATE_CONE, mjCNSTRSTATE_CONE
      ]
      ~unexpected:(fun _ ->
        failwith "unexpected mjtConstraintState element data type enum")


  (**  type of sensor *)
  type mjtSensor =
    | MjSENS_TOUCH
    | MjSENS_ACCELEROMETER
    | MjSENS_VELOCIMETER
    | MjSENS_GYRO
    | MjSENS_FORCE
    | MjSENS_TORQUE
    | MjSENS_MAGNETOMETER
    | MjSENS_RANGEFINDER
    | MjSENS_JOINTPOS
    | MjSENS_JOINTVEL
    | MjSENS_TENDONPOS
    | MjSENS_TENDONVEL
    | MjSENS_ACTUATORPOS
    | MjSENS_ACTUATORVEL
    | MjSENS_ACTUATORFRC
    | MjSENS_BALLQUAT
    | MjSENS_BALLANGVEL
    | MjSENS_JOINTLIMITPOS
    | MjSENS_JOINTLIMITVEL
    | MjSENS_JOINTLIMITFRC
    | MjSENS_TENDONLIMITPOS
    | MjSENS_TENDONLIMITVEL
    | MjSENS_TENDONLIMITFRC
    | MjSENS_FRAMEPOS
    | MjSENS_FRAMEQUAT
    | MjSENS_FRAMEXAXIS
    | MjSENS_FRAMEYAXIS
    | MjSENS_FRAMEZAXIS
    | MjSENS_FRAMELINVEL
    | MjSENS_FRAMEANGVEL
    | MjSENS_FRAMELINACC
    | MjSENS_FRAMEANGACC
    | MjSENS_SUBTREECOM
    | MjSENS_SUBTREELINVEL
    | MjSENS_SUBTREEANGMOM
    | MjSENS_USER

  let mjSENS_TOUCH = constant "mjSENS_TOUCH" int64_t
  let mjSENS_ACCELEROMETER = constant "mjSENS_ACCELEROMETER" int64_t
  let mjSENS_VELOCIMETER = constant "mjSENS_VELOCIMETER" int64_t
  let mjSENS_GYRO = constant "mjSENS_GYRO" int64_t
  let mjSENS_FORCE = constant "mjSENS_FORCE" int64_t
  let mjSENS_TORQUE = constant "mjSENS_TORQUE" int64_t
  let mjSENS_MAGNETOMETER = constant "mjSENS_MAGNETOMETER" int64_t
  let mjSENS_RANGEFINDER = constant "mjSENS_RANGEFINDER" int64_t
  let mjSENS_JOINTPOS = constant "mjSENS_JOINTPOS" int64_t
  let mjSENS_JOINTVEL = constant "mjSENS_JOINTVEL" int64_t
  let mjSENS_TENDONPOS = constant "mjSENS_TENDONPOS" int64_t
  let mjSENS_TENDONVEL = constant "mjSENS_TENDONVEL" int64_t
  let mjSENS_ACTUATORPOS = constant "mjSENS_ACTUATORPOS" int64_t
  let mjSENS_ACTUATORVEL = constant "mjSENS_ACTUATORVEL" int64_t
  let mjSENS_ACTUATORFRC = constant "mjSENS_ACTUATORFRC" int64_t
  let mjSENS_BALLQUAT = constant "mjSENS_BALLQUAT" int64_t
  let mjSENS_BALLANGVEL = constant "mjSENS_BALLANGVEL" int64_t
  let mjSENS_JOINTLIMITPOS = constant "mjSENS_JOINTLIMITPOS" int64_t
  let mjSENS_JOINTLIMITVEL = constant "mjSENS_JOINTLIMITVEL" int64_t
  let mjSENS_JOINTLIMITFRC = constant "mjSENS_JOINTLIMITFRC" int64_t
  let mjSENS_TENDONLIMITPOS = constant "mjSENS_TENDONLIMITPOS" int64_t
  let mjSENS_TENDONLIMITVEL = constant "mjSENS_TENDONLIMITVEL" int64_t
  let mjSENS_TENDONLIMITFRC = constant "mjSENS_TENDONLIMITFRC" int64_t
  let mjSENS_FRAMEPOS = constant "mjSENS_FRAMEPOS" int64_t
  let mjSENS_FRAMEQUAT = constant "mjSENS_FRAMEQUAT" int64_t
  let mjSENS_FRAMEXAXIS = constant "mjSENS_FRAMEXAXIS" int64_t
  let mjSENS_FRAMEYAXIS = constant "mjSENS_FRAMEYAXIS" int64_t
  let mjSENS_FRAMEZAXIS = constant "mjSENS_FRAMEZAXIS" int64_t
  let mjSENS_FRAMELINVEL = constant "mjSENS_FRAMELINVEL" int64_t
  let mjSENS_FRAMEANGVEL = constant "mjSENS_FRAMEANGVEL" int64_t
  let mjSENS_FRAMELINACC = constant "mjSENS_FRAMELINACC" int64_t
  let mjSENS_FRAMEANGACC = constant "mjSENS_FRAMEANGACC" int64_t
  let mjSENS_SUBTREECOM = constant "mjSENS_SUBTREECOM" int64_t
  let mjSENS_SUBTREELINVEL = constant "mjSENS_SUBTREELINVEL" int64_t
  let mjSENS_SUBTREEANGMOM = constant "mjSENS_SUBTREEANGMOM" int64_t
  let mjSENS_USER = constant "mjSENS_USER" int64_t

  let mjtSensor =
    S.enum
      "mjtSensor"
      ~typedef:true
      [ MjSENS_TOUCH, mjSENS_TOUCH
      ; MjSENS_ACCELEROMETER, mjSENS_ACCELEROMETER
      ; MjSENS_VELOCIMETER, mjSENS_VELOCIMETER
      ; MjSENS_GYRO, mjSENS_GYRO
      ; MjSENS_FORCE, mjSENS_FORCE
      ; MjSENS_TORQUE, mjSENS_TORQUE
      ; MjSENS_MAGNETOMETER, mjSENS_MAGNETOMETER
      ; MjSENS_RANGEFINDER, mjSENS_RANGEFINDER
      ; MjSENS_JOINTPOS, mjSENS_JOINTPOS
      ; MjSENS_JOINTVEL, mjSENS_JOINTVEL
      ; MjSENS_TENDONPOS, mjSENS_TENDONPOS
      ; MjSENS_TENDONVEL, mjSENS_TENDONVEL
      ; MjSENS_ACTUATORPOS, mjSENS_ACTUATORPOS
      ; MjSENS_ACTUATORVEL, mjSENS_ACTUATORVEL
      ; MjSENS_ACTUATORFRC, mjSENS_ACTUATORFRC
      ; MjSENS_BALLQUAT, mjSENS_BALLQUAT
      ; MjSENS_BALLANGVEL, mjSENS_BALLANGVEL
      ; MjSENS_JOINTLIMITPOS, mjSENS_JOINTLIMITPOS
      ; MjSENS_JOINTLIMITVEL, mjSENS_JOINTLIMITVEL
      ; MjSENS_JOINTLIMITFRC, mjSENS_JOINTLIMITFRC
      ; MjSENS_TENDONLIMITPOS, mjSENS_TENDONLIMITPOS
      ; MjSENS_TENDONLIMITVEL, mjSENS_TENDONLIMITVEL
      ; MjSENS_TENDONLIMITFRC, mjSENS_TENDONLIMITFRC
      ; MjSENS_FRAMEPOS, mjSENS_FRAMEPOS
      ; MjSENS_FRAMEQUAT, mjSENS_FRAMEQUAT
      ; MjSENS_FRAMEXAXIS, mjSENS_FRAMEXAXIS
      ; MjSENS_FRAMEYAXIS, mjSENS_FRAMEYAXIS
      ; MjSENS_FRAMEZAXIS, mjSENS_FRAMEZAXIS
      ; MjSENS_FRAMELINVEL, mjSENS_FRAMELINVEL
      ; MjSENS_FRAMEANGVEL, mjSENS_FRAMEANGVEL
      ; MjSENS_FRAMELINACC, mjSENS_FRAMELINACC
      ; MjSENS_FRAMEANGACC, mjSENS_FRAMEANGACC
      ; MjSENS_SUBTREECOM, mjSENS_SUBTREECOM
      ; MjSENS_SUBTREELINVEL, mjSENS_SUBTREELINVEL
      ; MjSENS_SUBTREEANGMOM, mjSENS_SUBTREEANGMOM
      ; MjSENS_USER, mjSENS_USER
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtSensor element data type enum")


  (**  computation stage *)
  type mjtStage =
    | MjSTAGE_NONE
    | MjSTAGE_POS
    | MjSTAGE_VEL
    | MjSTAGE_ACC

  let mjSTAGE_NONE = constant "mjSTAGE_NONE" int64_t
  let mjSTAGE_POS = constant "mjSTAGE_POS" int64_t
  let mjSTAGE_VEL = constant "mjSTAGE_VEL" int64_t
  let mjSTAGE_ACC = constant "mjSTAGE_ACC" int64_t

  let mjtStage =
    S.enum
      "mjtStage"
      ~typedef:true
      [ MjSTAGE_NONE, mjSTAGE_NONE
      ; MjSTAGE_POS, mjSTAGE_POS
      ; MjSTAGE_VEL, mjSTAGE_VEL
      ; MjSTAGE_ACC, mjSTAGE_ACC
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtStage element data type enum")


  (**  data type for sensors *)
  type mjtDataType =
    | MjDATATYPE_REAL
    | MjDATATYPE_POSITIVE
    | MjDATATYPE_AXIS
    | MjDATATYPE_QUATERNION

  let mjDATATYPE_REAL = constant "mjDATATYPE_REAL" int64_t
  let mjDATATYPE_POSITIVE = constant "mjDATATYPE_POSITIVE" int64_t
  let mjDATATYPE_AXIS = constant "mjDATATYPE_AXIS" int64_t
  let mjDATATYPE_QUATERNION = constant "mjDATATYPE_QUATERNION" int64_t

  let mjtDataType =
    S.enum
      "mjtDataType"
      ~typedef:true
      [ MjDATATYPE_REAL, mjDATATYPE_REAL
      ; MjDATATYPE_POSITIVE, mjDATATYPE_POSITIVE
      ; MjDATATYPE_AXIS, mjDATATYPE_AXIS
      ; MjDATATYPE_QUATERNION, mjDATATYPE_QUATERNION
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtDataType element data type enum")


  (**  mode for actuator length range computation *)
  type mjtLRMode =
    | MjLRMODE_NONE
    | MjLRMODE_MUSCLE
    | MjLRMODE_MUSCLEUSER
    | MjLRMODE_ALL

  let mjLRMODE_NONE = constant "mjLRMODE_NONE" int64_t
  let mjLRMODE_MUSCLE = constant "mjLRMODE_MUSCLE" int64_t
  let mjLRMODE_MUSCLEUSER = constant "mjLRMODE_MUSCLEUSER" int64_t
  let mjLRMODE_ALL = constant "mjLRMODE_ALL" int64_t

  let mjtLRMode =
    S.enum
      "mjtLRMode"
      ~typedef:true
      [ MjLRMODE_NONE, mjLRMODE_NONE
      ; MjLRMODE_MUSCLE, mjLRMODE_MUSCLE
      ; MjLRMODE_MUSCLEUSER, mjLRMODE_MUSCLEUSER
      ; MjLRMODE_ALL, mjLRMODE_ALL
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtLRMode element data type enum")


  type _mjLROpt

  let _mjLROpt : _mjLROpt structure typ = structure "_mjLROpt"

  (**  which actuators to process (mjtLRMode) *)
  let mjLROpt_mode = field _mjLROpt "mode" int

  (**  use existing length range if available *)
  let mjLROpt_useexisting = field _mjLROpt "useexisting" int

  (**  use joint and tendon limits if available *)
  let mjLROpt_uselimit = field _mjLROpt "uselimit" int

  (**  target acceleration used to compute force *)
  let mjLROpt_accel = field _mjLROpt "accel" mjtNum

  (**  maximum force; 0: no limit *)
  let mjLROpt_maxforce = field _mjLROpt "maxforce" mjtNum

  (**  time constant for velocity reduction; min 0.01 *)
  let mjLROpt_timeconst = field _mjLROpt "timeconst" mjtNum

  (**  simulation timestep; 0: use mjOption.timestep *)
  let mjLROpt_timestep = field _mjLROpt "timestep" mjtNum

  (**  total simulation time interval *)
  let mjLROpt_inttotal = field _mjLROpt "inttotal" mjtNum

  (**  evaluation time interval (at the end) *)
  let mjLROpt_inteval = field _mjLROpt "inteval" mjtNum

  (**  convergence tolerance (relative to range) *)
  let mjLROpt_tolrange = field _mjLROpt "tolrange" mjtNum

  let () = seal _mjLROpt

  type mjLROpt = _mjLROpt

  let mjLROpt = _mjLROpt

  type _mjVFS

  let _mjVFS : _mjVFS structure typ = structure "_mjVFS"

  (**  number of files present *)
  let mjVFS_nfile = field _mjVFS "nfile" int

  (**  file size in bytes *)
  let mjVFS_filesize = field _mjVFS "filesize" (ptr int)

  (**  buffer with file data *)
  let mjVFS_filedata = field _mjVFS "filedata" (ptr (ptr void))

  let () = seal _mjVFS

  type mjVFS = _mjVFS

  let mjVFS = _mjVFS

  type _mjOption

  let _mjOption : _mjOption structure typ = structure "_mjOption"

  (**  timestep *)
  let mjOption_timestep = field _mjOption "timestep" mjtNum

  (**  update rate for remote API (Hz) *)
  let mjOption_apirate = field _mjOption "apirate" mjtNum

  (**  ratio of friction-to-normal contact impedance *)
  let mjOption_impratio = field _mjOption "impratio" mjtNum

  (**  main solver tolerance *)
  let mjOption_tolerance = field _mjOption "tolerance" mjtNum

  (**  noslip solver tolerance *)
  let mjOption_noslip_tolerance = field _mjOption "noslip_tolerance" mjtNum

  (**  MPR solver tolerance *)
  let mjOption_mpr_tolerance = field _mjOption "mpr_tolerance" mjtNum

  (**  gravitational acceleration *)
  let mjOption_gravity = field _mjOption "gravity" (ptr mjtNum)

  (**  wind (for lift, drag and viscosity) *)
  let mjOption_wind = field _mjOption "wind" (ptr mjtNum)

  (**  global magnetic flux *)
  let mjOption_magnetic = field _mjOption "magnetic" (ptr mjtNum)

  (**  density of medium *)
  let mjOption_density = field _mjOption "density" mjtNum

  (**  viscosity of medium *)
  let mjOption_viscosity = field _mjOption "viscosity" mjtNum

  (**  margin *)
  let mjOption_o_margin = field _mjOption "o_margin" mjtNum

  (**  solref *)
  let mjOption_o_solref = field _mjOption "o_solref" (ptr mjtNum)

  (**  solimp *)
  let mjOption_o_solimp = field _mjOption "o_solimp" (ptr mjtNum)

  (**  integration mode (mjtIntegrator) *)
  let mjOption_integrator = field _mjOption "integrator" int

  (**  collision mode (mjtCollision) *)
  let mjOption_collision = field _mjOption "collision" int

  (**  type of friction cone (mjtCone) *)
  let mjOption_cone = field _mjOption "cone" int

  (**  type of Jacobian (mjtJacobian) *)
  let mjOption_jacobian = field _mjOption "jacobian" int

  (**  solver algorithm (mjtSolver) *)
  let mjOption_solver = field _mjOption "solver" int

  (**  maximum number of main solver iterations *)
  let mjOption_iterations = field _mjOption "iterations" int

  (**  maximum number of noslip solver iterations *)
  let mjOption_noslip_iterations = field _mjOption "noslip_iterations" int

  (**  maximum number of MPR solver iterations *)
  let mjOption_mpr_iterations = field _mjOption "mpr_iterations" int

  (**  bit flags for disabling standard features *)
  let mjOption_disableflags = field _mjOption "disableflags" int

  (**  bit flags for enabling optional features *)
  let mjOption_enableflags = field _mjOption "enableflags" int

  let () = seal _mjOption

  type mjOption = _mjOption

  let mjOption = _mjOption

  type _mjVisual

  let _mjVisual : _mjVisual structure typ = structure "_mjVisual"

  type mjVisual = _mjVisual

  let mjVisual = _mjVisual

  type _mjStatistic

  let _mjStatistic : _mjStatistic structure typ = structure "_mjStatistic"

  (**  mean diagonal inertia *)
  let mjStatistic_meaninertia = field _mjStatistic "meaninertia" mjtNum

  (**  mean body mass *)
  let mjStatistic_meanmass = field _mjStatistic "meanmass" mjtNum

  (**  mean body size *)
  let mjStatistic_meansize = field _mjStatistic "meansize" mjtNum

  (**  spatial extent *)
  let mjStatistic_extent = field _mjStatistic "extent" mjtNum

  (**  center of model *)
  let mjStatistic_center = field _mjStatistic "center" (ptr mjtNum)

  let () = seal _mjStatistic

  type mjStatistic = _mjStatistic

  let mjStatistic = _mjStatistic

  type _mjModel

  let _mjModel : _mjModel structure typ = structure "_mjModel"

  (**  number of generalized coordinates = dim(qpos) *)
  let mjModel_nq = field _mjModel "nq" int

  (**  number of degrees of freedom = dim(qvel) *)
  let mjModel_nv = field _mjModel "nv" int

  (**  number of actuators/controls = dim(ctrl) *)
  let mjModel_nu = field _mjModel "nu" int

  (**  number of activation states = dim(act) *)
  let mjModel_na = field _mjModel "na" int

  (**  number of bodies *)
  let mjModel_nbody = field _mjModel "nbody" int

  (**  number of joints *)
  let mjModel_njnt = field _mjModel "njnt" int

  (**  number of geoms *)
  let mjModel_ngeom = field _mjModel "ngeom" int

  (**  number of sites *)
  let mjModel_nsite = field _mjModel "nsite" int

  (**  number of cameras *)
  let mjModel_ncam = field _mjModel "ncam" int

  (**  number of lights *)
  let mjModel_nlight = field _mjModel "nlight" int

  (**  number of meshes *)
  let mjModel_nmesh = field _mjModel "nmesh" int

  (**  number of vertices in all meshes *)
  let mjModel_nmeshvert = field _mjModel "nmeshvert" int

  (**  number of vertices with texcoords in all meshes *)
  let mjModel_nmeshtexvert = field _mjModel "nmeshtexvert" int

  (**  number of triangular faces in all meshes *)
  let mjModel_nmeshface = field _mjModel "nmeshface" int

  (**  number of ints in mesh auxiliary data *)
  let mjModel_nmeshgraph = field _mjModel "nmeshgraph" int

  (**  number of skins *)
  let mjModel_nskin = field _mjModel "nskin" int

  (**  number of vertices in all skins *)
  let mjModel_nskinvert = field _mjModel "nskinvert" int

  (**  number of vertiex with texcoords in all skins *)
  let mjModel_nskintexvert = field _mjModel "nskintexvert" int

  (**  number of triangular faces in all skins *)
  let mjModel_nskinface = field _mjModel "nskinface" int

  (**  number of bones in all skins *)
  let mjModel_nskinbone = field _mjModel "nskinbone" int

  (**  number of vertices in all skin bones *)
  let mjModel_nskinbonevert = field _mjModel "nskinbonevert" int

  (**  number of heightfields *)
  let mjModel_nhfield = field _mjModel "nhfield" int

  (**  number of data points in all heightfields *)
  let mjModel_nhfielddata = field _mjModel "nhfielddata" int

  (**  number of textures *)
  let mjModel_ntex = field _mjModel "ntex" int

  (**  number of bytes in texture rgb data *)
  let mjModel_ntexdata = field _mjModel "ntexdata" int

  (**  number of materials *)
  let mjModel_nmat = field _mjModel "nmat" int

  (**  number of predefined geom pairs *)
  let mjModel_npair = field _mjModel "npair" int

  (**  number of excluded geom pairs *)
  let mjModel_nexclude = field _mjModel "nexclude" int

  (**  number of equality constraints *)
  let mjModel_neq = field _mjModel "neq" int

  (**  number of tendons *)
  let mjModel_ntendon = field _mjModel "ntendon" int

  (**  number of wrap objects in all tendon paths *)
  let mjModel_nwrap = field _mjModel "nwrap" int

  (**  number of sensors *)
  let mjModel_nsensor = field _mjModel "nsensor" int

  (**  number of numeric custom fields *)
  let mjModel_nnumeric = field _mjModel "nnumeric" int

  (**  number of mjtNums in all numeric fields *)
  let mjModel_nnumericdata = field _mjModel "nnumericdata" int

  (**  number of text custom fields *)
  let mjModel_ntext = field _mjModel "ntext" int

  (**  number of mjtBytes in all text fields *)
  let mjModel_ntextdata = field _mjModel "ntextdata" int

  (**  number of tuple custom fields *)
  let mjModel_ntuple = field _mjModel "ntuple" int

  (**  number of objects in all tuple fields *)
  let mjModel_ntupledata = field _mjModel "ntupledata" int

  (**  number of keyframes *)
  let mjModel_nkey = field _mjModel "nkey" int

  (**  number of mocap bodies *)
  let mjModel_nmocap = field _mjModel "nmocap" int

  (**  number of mjtNums in body_user *)
  let mjModel_nuser_body = field _mjModel "nuser_body" int

  (**  number of mjtNums in jnt_user *)
  let mjModel_nuser_jnt = field _mjModel "nuser_jnt" int

  (**  number of mjtNums in geom_user *)
  let mjModel_nuser_geom = field _mjModel "nuser_geom" int

  (**  number of mjtNums in site_user *)
  let mjModel_nuser_site = field _mjModel "nuser_site" int

  (**  number of mjtNums in cam_user *)
  let mjModel_nuser_cam = field _mjModel "nuser_cam" int

  (**  number of mjtNums in tendon_user *)
  let mjModel_nuser_tendon = field _mjModel "nuser_tendon" int

  (**  number of mjtNums in actuator_user *)
  let mjModel_nuser_actuator = field _mjModel "nuser_actuator" int

  (**  number of mjtNums in sensor_user *)
  let mjModel_nuser_sensor = field _mjModel "nuser_sensor" int

  (**  number of chars in all names *)
  let mjModel_nnames = field _mjModel "nnames" int

  (**  number of non-zeros in sparse inertia matrix *)
  let mjModel_nM = field _mjModel "nM" int

  (**  number of potential equality-constraint rows *)
  let mjModel_nemax = field _mjModel "nemax" int

  (**  number of available rows in constraint Jacobian *)
  let mjModel_njmax = field _mjModel "njmax" int

  (**  number of potential contacts in contact list *)
  let mjModel_nconmax = field _mjModel "nconmax" int

  (**  number of fields in mjData stack *)
  let mjModel_nstack = field _mjModel "nstack" int

  (**  number of extra fields in mjData *)
  let mjModel_nuserdata = field _mjModel "nuserdata" int

  (**  number of fields in sensor data vector *)
  let mjModel_nsensordata = field _mjModel "nsensordata" int

  (**  number of bytes in buffer *)
  let mjModel_nbuffer = field _mjModel "nbuffer" int

  (**  physics options *)
  let mjModel_opt = field _mjModel "opt" mjOption

  (**  visualization options *)
  let mjModel_vis = field _mjModel "vis" mjVisual

  (**  model statistics *)
  let mjModel_stat = field _mjModel "stat" mjStatistic

  (**  main buffer; all pointers point in it    (nbuffer) *)
  let mjModel_buffer = field _mjModel "buffer" (ptr void)

  (**  qpos values at default pose              (nq x 1) *)
  let mjModel_qpos0 = field _mjModel "qpos0" (ptr mjtNum)

  (**  reference pose for springs               (nq x 1) *)
  let mjModel_qpos_spring = field _mjModel "qpos_spring" (ptr mjtNum)

  (**  id of body's parent                      (nbody x 1) *)
  let mjModel_body_parentid = field _mjModel "body_parentid" (ptr int)

  (**  id of root above body                    (nbody x 1) *)
  let mjModel_body_rootid = field _mjModel "body_rootid" (ptr int)

  (**  id of body that this body is welded to   (nbody x 1) *)
  let mjModel_body_weldid = field _mjModel "body_weldid" (ptr int)

  (**  id of mocap data; -1: none               (nbody x 1) *)
  let mjModel_body_mocapid = field _mjModel "body_mocapid" (ptr int)

  (**  number of joints for this body           (nbody x 1) *)
  let mjModel_body_jntnum = field _mjModel "body_jntnum" (ptr int)

  (**  start addr of joints; -1: no joints      (nbody x 1) *)
  let mjModel_body_jntadr = field _mjModel "body_jntadr" (ptr int)

  (**  number of motion degrees of freedom      (nbody x 1) *)
  let mjModel_body_dofnum = field _mjModel "body_dofnum" (ptr int)

  (**  start addr of dofs; -1: no dofs          (nbody x 1) *)
  let mjModel_body_dofadr = field _mjModel "body_dofadr" (ptr int)

  (**  number of geoms                          (nbody x 1) *)
  let mjModel_body_geomnum = field _mjModel "body_geomnum" (ptr int)

  (**  start addr of geoms; -1: no geoms        (nbody x 1) *)
  let mjModel_body_geomadr = field _mjModel "body_geomadr" (ptr int)

  (**  body is simple (has diagonal M)          (nbody x 1) *)
  let mjModel_body_simple = field _mjModel "body_simple" (ptr mjtByte)

  (**  inertial frame is same as body frame     (nbody x 1) *)
  let mjModel_body_sameframe = field _mjModel "body_sameframe" (ptr mjtByte)

  (**  position offset rel. to parent body      (nbody x 3) *)
  let mjModel_body_pos = field _mjModel "body_pos" (ptr mjtNum)

  (**  orientation offset rel. to parent body   (nbody x 4) *)
  let mjModel_body_quat = field _mjModel "body_quat" (ptr mjtNum)

  (**  local position of center of mass         (nbody x 3) *)
  let mjModel_body_ipos = field _mjModel "body_ipos" (ptr mjtNum)

  (**  local orientation of inertia ellipsoid   (nbody x 4) *)
  let mjModel_body_iquat = field _mjModel "body_iquat" (ptr mjtNum)

  (**  mass                                     (nbody x 1) *)
  let mjModel_body_mass = field _mjModel "body_mass" (ptr mjtNum)

  (**  mass of subtree starting at this body    (nbody x 1) *)
  let mjModel_body_subtreemass = field _mjModel "body_subtreemass" (ptr mjtNum)

  (**  diagonal inertia in ipos/iquat frame     (nbody x 3) *)
  let mjModel_body_inertia = field _mjModel "body_inertia" (ptr mjtNum)

  (**  mean inv inert in qpos0 (trn, rot)       (nbody x 2) *)
  let mjModel_body_invweight0 = field _mjModel "body_invweight0" (ptr mjtNum)

  (**  user data                                (nbody x nuser_body) *)
  let mjModel_body_user = field _mjModel "body_user" (ptr mjtNum)

  (**  type of joint (mjtJoint)                 (njnt x 1) *)
  let mjModel_jnt_type = field _mjModel "jnt_type" (ptr int)

  (**  start addr in 'qpos' for joint's data    (njnt x 1) *)
  let mjModel_jnt_qposadr = field _mjModel "jnt_qposadr" (ptr int)

  (**  start addr in 'qvel' for joint's data    (njnt x 1) *)
  let mjModel_jnt_dofadr = field _mjModel "jnt_dofadr" (ptr int)

  (**  id of joint's body                       (njnt x 1) *)
  let mjModel_jnt_bodyid = field _mjModel "jnt_bodyid" (ptr int)

  (**  group for visibility                     (njnt x 1) *)
  let mjModel_jnt_group = field _mjModel "jnt_group" (ptr int)

  (**  does joint have limits                   (njnt x 1) *)
  let mjModel_jnt_limited = field _mjModel "jnt_limited" (ptr mjtByte)

  (**  constraint solver reference: limit       (njnt x mjNREF) *)
  let mjModel_jnt_solref = field _mjModel "jnt_solref" (ptr mjtNum)

  (**  constraint solver impedance: limit       (njnt x mjNIMP) *)
  let mjModel_jnt_solimp = field _mjModel "jnt_solimp" (ptr mjtNum)

  (**  local anchor position                    (njnt x 3) *)
  let mjModel_jnt_pos = field _mjModel "jnt_pos" (ptr mjtNum)

  (**  local joint axis                         (njnt x 3) *)
  let mjModel_jnt_axis = field _mjModel "jnt_axis" (ptr mjtNum)

  (**  stiffness coefficient                    (njnt x 1) *)
  let mjModel_jnt_stiffness = field _mjModel "jnt_stiffness" (ptr mjtNum)

  (**  joint limits                             (njnt x 2) *)
  let mjModel_jnt_range = field _mjModel "jnt_range" (ptr mjtNum)

  (**  min distance for limit detection         (njnt x 1) *)
  let mjModel_jnt_margin = field _mjModel "jnt_margin" (ptr mjtNum)

  (**  user data                                (njnt x nuser_jnt) *)
  let mjModel_jnt_user = field _mjModel "jnt_user" (ptr mjtNum)

  (**  id of dof's body                         (nv x 1) *)
  let mjModel_dof_bodyid = field _mjModel "dof_bodyid" (ptr int)

  (**  id of dof's joint                        (nv x 1) *)
  let mjModel_dof_jntid = field _mjModel "dof_jntid" (ptr int)

  (**  id of dof's parent; -1: none             (nv x 1) *)
  let mjModel_dof_parentid = field _mjModel "dof_parentid" (ptr int)

  (**  dof address in M-diagonal                (nv x 1) *)
  let mjModel_dof_Madr = field _mjModel "dof_Madr" (ptr int)

  (**  number of consecutive simple dofs        (nv x 1) *)
  let mjModel_dof_simplenum = field _mjModel "dof_simplenum" (ptr int)

  (**  constraint solver reference:frictionloss (nv x mjNREF) *)
  let mjModel_dof_solref = field _mjModel "dof_solref" (ptr mjtNum)

  (**  constraint solver impedance:frictionloss (nv x mjNIMP) *)
  let mjModel_dof_solimp = field _mjModel "dof_solimp" (ptr mjtNum)

  (**  dof friction loss                        (nv x 1) *)
  let mjModel_dof_frictionloss = field _mjModel "dof_frictionloss" (ptr mjtNum)

  (**  dof armature inertia/mass                (nv x 1) *)
  let mjModel_dof_armature = field _mjModel "dof_armature" (ptr mjtNum)

  (**  damping coefficient                      (nv x 1) *)
  let mjModel_dof_damping = field _mjModel "dof_damping" (ptr mjtNum)

  (**  diag. inverse inertia in qpos0           (nv x 1) *)
  let mjModel_dof_invweight0 = field _mjModel "dof_invweight0" (ptr mjtNum)

  (**  diag. inertia in qpos0                   (nv x 1) *)
  let mjModel_dof_M0 = field _mjModel "dof_M0" (ptr mjtNum)

  (**  geometric type (mjtGeom)                 (ngeom x 1) *)
  let mjModel_geom_type = field _mjModel "geom_type" (ptr int)

  (**  geom contact type                        (ngeom x 1) *)
  let mjModel_geom_contype = field _mjModel "geom_contype" (ptr int)

  (**  geom contact affinity                    (ngeom x 1) *)
  let mjModel_geom_conaffinity = field _mjModel "geom_conaffinity" (ptr int)

  (**  contact dimensionality (1, 3, 4, 6)      (ngeom x 1) *)
  let mjModel_geom_condim = field _mjModel "geom_condim" (ptr int)

  (**  id of geom's body                        (ngeom x 1) *)
  let mjModel_geom_bodyid = field _mjModel "geom_bodyid" (ptr int)

  (**  id of geom's mesh/hfield (-1: none)      (ngeom x 1) *)
  let mjModel_geom_dataid = field _mjModel "geom_dataid" (ptr int)

  (**  material id for rendering                (ngeom x 1) *)
  let mjModel_geom_matid = field _mjModel "geom_matid" (ptr int)

  (**  group for visibility                     (ngeom x 1) *)
  let mjModel_geom_group = field _mjModel "geom_group" (ptr int)

  (**  geom contact priority                    (ngeom x 1) *)
  let mjModel_geom_priority = field _mjModel "geom_priority" (ptr int)

  (**  same as body frame (1) or iframe (2)     (ngeom x 1) *)
  let mjModel_geom_sameframe = field _mjModel "geom_sameframe" (ptr mjtByte)

  (**  mixing coef for solref/imp in geom pair  (ngeom x 1) *)
  let mjModel_geom_solmix = field _mjModel "geom_solmix" (ptr mjtNum)

  (**  constraint solver reference: contact     (ngeom x mjNREF) *)
  let mjModel_geom_solref = field _mjModel "geom_solref" (ptr mjtNum)

  (**  constraint solver impedance: contact     (ngeom x mjNIMP) *)
  let mjModel_geom_solimp = field _mjModel "geom_solimp" (ptr mjtNum)

  (**  geom-specific size parameters            (ngeom x 3) *)
  let mjModel_geom_size = field _mjModel "geom_size" (ptr mjtNum)

  (**  radius of bounding sphere                (ngeom x 1) *)
  let mjModel_geom_rbound = field _mjModel "geom_rbound" (ptr mjtNum)

  (**  local position offset rel. to body       (ngeom x 3) *)
  let mjModel_geom_pos = field _mjModel "geom_pos" (ptr mjtNum)

  (**  local orientation offset rel. to body    (ngeom x 4) *)
  let mjModel_geom_quat = field _mjModel "geom_quat" (ptr mjtNum)

  (**  friction for (slide, spin, roll)         (ngeom x 3) *)
  let mjModel_geom_friction = field _mjModel "geom_friction" (ptr mjtNum)

  (**  detect contact if dist<margin            (ngeom x 1) *)
  let mjModel_geom_margin = field _mjModel "geom_margin" (ptr mjtNum)

  (**  include in solver if dist<margin-gap     (ngeom x 1) *)
  let mjModel_geom_gap = field _mjModel "geom_gap" (ptr mjtNum)

  (**  user data                                (ngeom x nuser_geom) *)
  let mjModel_geom_user = field _mjModel "geom_user" (ptr mjtNum)

  (**  rgba when material is omitted            (ngeom x 4) *)
  let mjModel_geom_rgba = field _mjModel "geom_rgba" (ptr float)

  (**  geom type for rendering (mjtGeom)        (nsite x 1) *)
  let mjModel_site_type = field _mjModel "site_type" (ptr int)

  (**  id of site's body                        (nsite x 1) *)
  let mjModel_site_bodyid = field _mjModel "site_bodyid" (ptr int)

  (**  material id for rendering                (nsite x 1) *)
  let mjModel_site_matid = field _mjModel "site_matid" (ptr int)

  (**  group for visibility                     (nsite x 1) *)
  let mjModel_site_group = field _mjModel "site_group" (ptr int)

  (**  same as body frame (1) or iframe (2)     (nsite x 1) *)
  let mjModel_site_sameframe = field _mjModel "site_sameframe" (ptr mjtByte)

  (**  geom size for rendering                  (nsite x 3) *)
  let mjModel_site_size = field _mjModel "site_size" (ptr mjtNum)

  (**  local position offset rel. to body       (nsite x 3) *)
  let mjModel_site_pos = field _mjModel "site_pos" (ptr mjtNum)

  (**  local orientation offset rel. to body    (nsite x 4) *)
  let mjModel_site_quat = field _mjModel "site_quat" (ptr mjtNum)

  (**  user data                                (nsite x nuser_site) *)
  let mjModel_site_user = field _mjModel "site_user" (ptr mjtNum)

  (**  rgba when material is omitted            (nsite x 4) *)
  let mjModel_site_rgba = field _mjModel "site_rgba" (ptr float)

  (**  camera tracking mode (mjtCamLight)       (ncam x 1) *)
  let mjModel_cam_mode = field _mjModel "cam_mode" (ptr int)

  (**  id of camera's body                      (ncam x 1) *)
  let mjModel_cam_bodyid = field _mjModel "cam_bodyid" (ptr int)

  (**  id of targeted body; -1: none            (ncam x 1) *)
  let mjModel_cam_targetbodyid = field _mjModel "cam_targetbodyid" (ptr int)

  (**  position rel. to body frame              (ncam x 3) *)
  let mjModel_cam_pos = field _mjModel "cam_pos" (ptr mjtNum)

  (**  orientation rel. to body frame           (ncam x 4) *)
  let mjModel_cam_quat = field _mjModel "cam_quat" (ptr mjtNum)

  (**  global position rel. to sub-com in qpos0 (ncam x 3) *)
  let mjModel_cam_poscom0 = field _mjModel "cam_poscom0" (ptr mjtNum)

  (**  global position rel. to body in qpos0    (ncam x 3) *)
  let mjModel_cam_pos0 = field _mjModel "cam_pos0" (ptr mjtNum)

  (**  global orientation in qpos0              (ncam x 9) *)
  let mjModel_cam_mat0 = field _mjModel "cam_mat0" (ptr mjtNum)

  (**  y-field of view (deg)                    (ncam x 1) *)
  let mjModel_cam_fovy = field _mjModel "cam_fovy" (ptr mjtNum)

  (**  inter-pupilary distance                  (ncam x 1) *)
  let mjModel_cam_ipd = field _mjModel "cam_ipd" (ptr mjtNum)

  (**  user data                                (ncam x nuser_cam) *)
  let mjModel_cam_user = field _mjModel "cam_user" (ptr mjtNum)

  (**  light tracking mode (mjtCamLight)        (nlight x 1) *)
  let mjModel_light_mode = field _mjModel "light_mode" (ptr int)

  (**  id of light's body                       (nlight x 1) *)
  let mjModel_light_bodyid = field _mjModel "light_bodyid" (ptr int)

  (**  id of targeted body; -1: none            (nlight x 1) *)
  let mjModel_light_targetbodyid = field _mjModel "light_targetbodyid" (ptr int)

  (**  directional light                        (nlight x 1) *)
  let mjModel_light_directional = field _mjModel "light_directional" (ptr mjtByte)

  (**  does light cast shadows                  (nlight x 1) *)
  let mjModel_light_castshadow = field _mjModel "light_castshadow" (ptr mjtByte)

  (**  is light on                              (nlight x 1) *)
  let mjModel_light_active = field _mjModel "light_active" (ptr mjtByte)

  (**  position rel. to body frame              (nlight x 3) *)
  let mjModel_light_pos = field _mjModel "light_pos" (ptr mjtNum)

  (**  direction rel. to body frame             (nlight x 3) *)
  let mjModel_light_dir = field _mjModel "light_dir" (ptr mjtNum)

  (**  global position rel. to sub-com in qpos0 (nlight x 3) *)
  let mjModel_light_poscom0 = field _mjModel "light_poscom0" (ptr mjtNum)

  (**  global position rel. to body in qpos0    (nlight x 3) *)
  let mjModel_light_pos0 = field _mjModel "light_pos0" (ptr mjtNum)

  (**  global direction in qpos0                (nlight x 3) *)
  let mjModel_light_dir0 = field _mjModel "light_dir0" (ptr mjtNum)

  (**  OpenGL attenuation (quadratic model)     (nlight x 3) *)
  let mjModel_light_attenuation = field _mjModel "light_attenuation" (ptr float)

  (**  OpenGL cutoff                            (nlight x 1) *)
  let mjModel_light_cutoff = field _mjModel "light_cutoff" (ptr float)

  (**  OpenGL exponent                          (nlight x 1) *)
  let mjModel_light_exponent = field _mjModel "light_exponent" (ptr float)

  (**  ambient rgb (alpha=1)                    (nlight x 3) *)
  let mjModel_light_ambient = field _mjModel "light_ambient" (ptr float)

  (**  diffuse rgb (alpha=1)                    (nlight x 3) *)
  let mjModel_light_diffuse = field _mjModel "light_diffuse" (ptr float)

  (**  specular rgb (alpha=1)                   (nlight x 3) *)
  let mjModel_light_specular = field _mjModel "light_specular" (ptr float)

  (**  first vertex address                     (nmesh x 1) *)
  let mjModel_mesh_vertadr = field _mjModel "mesh_vertadr" (ptr int)

  (**  number of vertices                       (nmesh x 1) *)
  let mjModel_mesh_vertnum = field _mjModel "mesh_vertnum" (ptr int)

  (**  texcoord data address; -1: no texcoord   (nmesh x 1) *)
  let mjModel_mesh_texcoordadr = field _mjModel "mesh_texcoordadr" (ptr int)

  (**  first face address                       (nmesh x 1) *)
  let mjModel_mesh_faceadr = field _mjModel "mesh_faceadr" (ptr int)

  (**  number of faces                          (nmesh x 1) *)
  let mjModel_mesh_facenum = field _mjModel "mesh_facenum" (ptr int)

  (**  graph data address; -1: no graph         (nmesh x 1) *)
  let mjModel_mesh_graphadr = field _mjModel "mesh_graphadr" (ptr int)

  (**  vertex positions for all meshe           (nmeshvert x 3) *)
  let mjModel_mesh_vert = field _mjModel "mesh_vert" (ptr float)

  (**  vertex normals for all meshes            (nmeshvert x 3) *)
  let mjModel_mesh_normal = field _mjModel "mesh_normal" (ptr float)

  (**  vertex texcoords for all meshes          (nmeshtexvert x 2) *)
  let mjModel_mesh_texcoord = field _mjModel "mesh_texcoord" (ptr float)

  (**  triangle face data                       (nmeshface x 3) *)
  let mjModel_mesh_face = field _mjModel "mesh_face" (ptr int)

  (**  convex graph data                        (nmeshgraph x 1) *)
  let mjModel_mesh_graph = field _mjModel "mesh_graph" (ptr int)

  (**  skin material id; -1: none               (nskin x 1) *)
  let mjModel_skin_matid = field _mjModel "skin_matid" (ptr int)

  (**  skin rgba                                (nskin x 4) *)
  let mjModel_skin_rgba = field _mjModel "skin_rgba" (ptr float)

  (**  inflate skin in normal direction         (nskin x 1) *)
  let mjModel_skin_inflate = field _mjModel "skin_inflate" (ptr float)

  (**  first vertex address                     (nskin x 1) *)
  let mjModel_skin_vertadr = field _mjModel "skin_vertadr" (ptr int)

  (**  number of vertices                       (nskin x 1) *)
  let mjModel_skin_vertnum = field _mjModel "skin_vertnum" (ptr int)

  (**  texcoord data address; -1: no texcoord   (nskin x 1) *)
  let mjModel_skin_texcoordadr = field _mjModel "skin_texcoordadr" (ptr int)

  (**  first face address                       (nskin x 1) *)
  let mjModel_skin_faceadr = field _mjModel "skin_faceadr" (ptr int)

  (**  number of faces                          (nskin x 1) *)
  let mjModel_skin_facenum = field _mjModel "skin_facenum" (ptr int)

  (**  first bone in skin                       (nskin x 1) *)
  let mjModel_skin_boneadr = field _mjModel "skin_boneadr" (ptr int)

  (**  number of bones in skin                  (nskin x 1) *)
  let mjModel_skin_bonenum = field _mjModel "skin_bonenum" (ptr int)

  (**  vertex positions for all skin meshes     (nskinvert x 3) *)
  let mjModel_skin_vert = field _mjModel "skin_vert" (ptr float)

  (**  vertex texcoords for all skin meshes     (nskintexvert x 2) *)
  let mjModel_skin_texcoord = field _mjModel "skin_texcoord" (ptr float)

  (**  triangle faces for all skin meshes       (nskinface x 3) *)
  let mjModel_skin_face = field _mjModel "skin_face" (ptr int)

  (**  first vertex in each bone                (nskinbone x 1) *)
  let mjModel_skin_bonevertadr = field _mjModel "skin_bonevertadr" (ptr int)

  (**  number of vertices in each bone          (nskinbone x 1) *)
  let mjModel_skin_bonevertnum = field _mjModel "skin_bonevertnum" (ptr int)

  (**  bind pos of each bone                    (nskinbone x 3) *)
  let mjModel_skin_bonebindpos = field _mjModel "skin_bonebindpos" (ptr float)

  (**  bind quat of each bone                   (nskinbone x 4) *)
  let mjModel_skin_bonebindquat = field _mjModel "skin_bonebindquat" (ptr float)

  (**  body id of each bone                     (nskinbone x 1) *)
  let mjModel_skin_bonebodyid = field _mjModel "skin_bonebodyid" (ptr int)

  (**  mesh ids of vertices in each bone        (nskinbonevert x 1) *)
  let mjModel_skin_bonevertid = field _mjModel "skin_bonevertid" (ptr int)

  (**  weights of vertices in each bone         (nskinbonevert x 1) *)
  let mjModel_skin_bonevertweight = field _mjModel "skin_bonevertweight" (ptr float)

  (**  (x, y, z_top, z_bottom)                  (nhfield x 4) *)
  let mjModel_hfield_size = field _mjModel "hfield_size" (ptr mjtNum)

  (**  number of rows in grid                   (nhfield x 1) *)
  let mjModel_hfield_nrow = field _mjModel "hfield_nrow" (ptr int)

  (**  number of columns in grid                (nhfield x 1) *)
  let mjModel_hfield_ncol = field _mjModel "hfield_ncol" (ptr int)

  (**  address in hfield_data                   (nhfield x 1) *)
  let mjModel_hfield_adr = field _mjModel "hfield_adr" (ptr int)

  (**  elevation data                           (nhfielddata x 1) *)
  let mjModel_hfield_data = field _mjModel "hfield_data" (ptr float)

  (**  texture type (mjtTexture)                (ntex x 1) *)
  let mjModel_tex_type = field _mjModel "tex_type" (ptr int)

  (**  number of rows in texture image          (ntex x 1) *)
  let mjModel_tex_height = field _mjModel "tex_height" (ptr int)

  (**  number of columns in texture image       (ntex x 1) *)
  let mjModel_tex_width = field _mjModel "tex_width" (ptr int)

  (**  address in rgb                           (ntex x 1) *)
  let mjModel_tex_adr = field _mjModel "tex_adr" (ptr int)

  (**  rgb (alpha = 1)                          (ntexdata x 1) *)
  let mjModel_tex_rgb = field _mjModel "tex_rgb" (ptr mjtByte)

  (**  texture id; -1: none                     (nmat x 1) *)
  let mjModel_mat_texid = field _mjModel "mat_texid" (ptr int)

  (**  make texture cube uniform                (nmat x 1) *)
  let mjModel_mat_texuniform = field _mjModel "mat_texuniform" (ptr mjtByte)

  (**  texture repetition for 2d mapping        (nmat x 2) *)
  let mjModel_mat_texrepeat = field _mjModel "mat_texrepeat" (ptr float)

  (**  emission (x rgb)                         (nmat x 1) *)
  let mjModel_mat_emission = field _mjModel "mat_emission" (ptr float)

  (**  specular (x white)                       (nmat x 1) *)
  let mjModel_mat_specular = field _mjModel "mat_specular" (ptr float)

  (**  shininess coef                           (nmat x 1) *)
  let mjModel_mat_shininess = field _mjModel "mat_shininess" (ptr float)

  (**  reflectance (0: disable)                 (nmat x 1) *)
  let mjModel_mat_reflectance = field _mjModel "mat_reflectance" (ptr float)

  (**  rgba                                     (nmat x 4) *)
  let mjModel_mat_rgba = field _mjModel "mat_rgba" (ptr float)

  (**  contact dimensionality                   (npair x 1) *)
  let mjModel_pair_dim = field _mjModel "pair_dim" (ptr int)

  (**  id of geom1                              (npair x 1) *)
  let mjModel_pair_geom1 = field _mjModel "pair_geom1" (ptr int)

  (**  id of geom2                              (npair x 1) *)
  let mjModel_pair_geom2 = field _mjModel "pair_geom2" (ptr int)

  (**  (body1+1)<<16 + body2+1                  (npair x 1) *)
  let mjModel_pair_signature = field _mjModel "pair_signature" (ptr int)

  (**  constraint solver reference: contact     (npair x mjNREF) *)
  let mjModel_pair_solref = field _mjModel "pair_solref" (ptr mjtNum)

  (**  constraint solver impedance: contact     (npair x mjNIMP) *)
  let mjModel_pair_solimp = field _mjModel "pair_solimp" (ptr mjtNum)

  (**  detect contact if dist<margin            (npair x 1) *)
  let mjModel_pair_margin = field _mjModel "pair_margin" (ptr mjtNum)

  (**  include in solver if dist<margin-gap     (npair x 1) *)
  let mjModel_pair_gap = field _mjModel "pair_gap" (ptr mjtNum)

  (**  tangent1, 2, spin, roll1, 2              (npair x 5) *)
  let mjModel_pair_friction = field _mjModel "pair_friction" (ptr mjtNum)

  (**  (body1+1)<<16 + body2+1                  (nexclude x 1) *)
  let mjModel_exclude_signature = field _mjModel "exclude_signature" (ptr int)

  (**  constraint type (mjtEq)                  (neq x 1) *)
  let mjModel_eq_type = field _mjModel "eq_type" (ptr int)

  (**  id of object 1                           (neq x 1) *)
  let mjModel_eq_obj1id = field _mjModel "eq_obj1id" (ptr int)

  (**  id of object 2                           (neq x 1) *)
  let mjModel_eq_obj2id = field _mjModel "eq_obj2id" (ptr int)

  (**  enable/disable constraint                (neq x 1) *)
  let mjModel_eq_active = field _mjModel "eq_active" (ptr mjtByte)

  (**  constraint solver reference              (neq x mjNREF) *)
  let mjModel_eq_solref = field _mjModel "eq_solref" (ptr mjtNum)

  (**  constraint solver impedance              (neq x mjNIMP) *)
  let mjModel_eq_solimp = field _mjModel "eq_solimp" (ptr mjtNum)

  (**  numeric data for constraint              (neq x mjNEQDATA) *)
  let mjModel_eq_data = field _mjModel "eq_data" (ptr mjtNum)

  (**  address of first object in tendon's path (ntendon x 1) *)
  let mjModel_tendon_adr = field _mjModel "tendon_adr" (ptr int)

  (**  number of objects in tendon's path       (ntendon x 1) *)
  let mjModel_tendon_num = field _mjModel "tendon_num" (ptr int)

  (**  material id for rendering                (ntendon x 1) *)
  let mjModel_tendon_matid = field _mjModel "tendon_matid" (ptr int)

  (**  group for visibility                     (ntendon x 1) *)
  let mjModel_tendon_group = field _mjModel "tendon_group" (ptr int)

  (**  does tendon have length limits           (ntendon x 1) *)
  let mjModel_tendon_limited = field _mjModel "tendon_limited" (ptr mjtByte)

  (**  width for rendering                      (ntendon x 1) *)
  let mjModel_tendon_width = field _mjModel "tendon_width" (ptr mjtNum)

  (**  constraint solver reference: limit       (ntendon x mjNREF) *)
  let mjModel_tendon_solref_lim = field _mjModel "tendon_solref_lim" (ptr mjtNum)

  (**  constraint solver impedance: limit       (ntendon x mjNIMP) *)
  let mjModel_tendon_solimp_lim = field _mjModel "tendon_solimp_lim" (ptr mjtNum)

  (**  constraint solver reference: friction    (ntendon x mjNREF) *)
  let mjModel_tendon_solref_fri = field _mjModel "tendon_solref_fri" (ptr mjtNum)

  (**  constraint solver impedance: friction    (ntendon x mjNIMP) *)
  let mjModel_tendon_solimp_fri = field _mjModel "tendon_solimp_fri" (ptr mjtNum)

  (**  tendon length limits                     (ntendon x 2) *)
  let mjModel_tendon_range = field _mjModel "tendon_range" (ptr mjtNum)

  (**  min distance for limit detection         (ntendon x 1) *)
  let mjModel_tendon_margin = field _mjModel "tendon_margin" (ptr mjtNum)

  (**  stiffness coefficient                    (ntendon x 1) *)
  let mjModel_tendon_stiffness = field _mjModel "tendon_stiffness" (ptr mjtNum)

  (**  damping coefficient                      (ntendon x 1) *)
  let mjModel_tendon_damping = field _mjModel "tendon_damping" (ptr mjtNum)

  (**  loss due to friction                     (ntendon x 1) *)
  let mjModel_tendon_frictionloss = field _mjModel "tendon_frictionloss" (ptr mjtNum)

  (**  tendon length in qpos_spring             (ntendon x 1) *)
  let mjModel_tendon_lengthspring = field _mjModel "tendon_lengthspring" (ptr mjtNum)

  (**  tendon length in qpos0                   (ntendon x 1) *)
  let mjModel_tendon_length0 = field _mjModel "tendon_length0" (ptr mjtNum)

  (**  inv. weight in qpos0                     (ntendon x 1) *)
  let mjModel_tendon_invweight0 = field _mjModel "tendon_invweight0" (ptr mjtNum)

  (**  user data                                (ntendon x nuser_tendon) *)
  let mjModel_tendon_user = field _mjModel "tendon_user" (ptr mjtNum)

  (**  rgba when material is omitted            (ntendon x 4) *)
  let mjModel_tendon_rgba = field _mjModel "tendon_rgba" (ptr float)

  (**  wrap object type (mjtWrap)               (nwrap x 1) *)
  let mjModel_wrap_type = field _mjModel "wrap_type" (ptr int)

  (**  object id: geom, site, joint             (nwrap x 1) *)
  let mjModel_wrap_objid = field _mjModel "wrap_objid" (ptr int)

  (**  divisor, joint coef, or site id          (nwrap x 1) *)
  let mjModel_wrap_prm = field _mjModel "wrap_prm" (ptr mjtNum)

  (**  transmission type (mjtTrn)               (nu x 1) *)
  let mjModel_actuator_trntype = field _mjModel "actuator_trntype" (ptr int)

  (**  dynamics type (mjtDyn)                   (nu x 1) *)
  let mjModel_actuator_dyntype = field _mjModel "actuator_dyntype" (ptr int)

  (**  gain type (mjtGain)                      (nu x 1) *)
  let mjModel_actuator_gaintype = field _mjModel "actuator_gaintype" (ptr int)

  (**  bias type (mjtBias)                      (nu x 1) *)
  let mjModel_actuator_biastype = field _mjModel "actuator_biastype" (ptr int)

  (**  transmission id: joint, tendon, site     (nu x 2) *)
  let mjModel_actuator_trnid = field _mjModel "actuator_trnid" (ptr int)

  (**  group for visibility                     (nu x 1) *)
  let mjModel_actuator_group = field _mjModel "actuator_group" (ptr int)

  (**  is control limited                       (nu x 1) *)
  let mjModel_actuator_ctrllimited = field _mjModel "actuator_ctrllimited" (ptr mjtByte)

  (**  is force limited                         (nu x 1) *)
  let mjModel_actuator_forcelimited = field _mjModel "actuator_forcelimited" (ptr mjtByte)

  (**  dynamics parameters                      (nu x mjNDYN) *)
  let mjModel_actuator_dynprm = field _mjModel "actuator_dynprm" (ptr mjtNum)

  (**  gain parameters                          (nu x mjNGAIN) *)
  let mjModel_actuator_gainprm = field _mjModel "actuator_gainprm" (ptr mjtNum)

  (**  bias parameters                          (nu x mjNBIAS) *)
  let mjModel_actuator_biasprm = field _mjModel "actuator_biasprm" (ptr mjtNum)

  (**  range of controls                        (nu x 2) *)
  let mjModel_actuator_ctrlrange = field _mjModel "actuator_ctrlrange" (ptr mjtNum)

  (**  range of forces                          (nu x 2) *)
  let mjModel_actuator_forcerange = field _mjModel "actuator_forcerange" (ptr mjtNum)

  (**  scale length and transmitted force       (nu x 6) *)
  let mjModel_actuator_gear = field _mjModel "actuator_gear" (ptr mjtNum)

  (**  crank length for slider-crank            (nu x 1) *)
  let mjModel_actuator_cranklength = field _mjModel "actuator_cranklength" (ptr mjtNum)

  (**  acceleration from unit force in qpos0    (nu x 1) *)
  let mjModel_actuator_acc0 = field _mjModel "actuator_acc0" (ptr mjtNum)

  (**  actuator length in qpos0                 (nu x 1) *)
  let mjModel_actuator_length0 = field _mjModel "actuator_length0" (ptr mjtNum)

  (**  feasible actuator length range           (nu x 2) *)
  let mjModel_actuator_lengthrange = field _mjModel "actuator_lengthrange" (ptr mjtNum)

  (**  user data                                (nu x nuser_actuator) *)
  let mjModel_actuator_user = field _mjModel "actuator_user" (ptr mjtNum)

  (**  sensor type (mjtSensor)                  (nsensor x 1) *)
  let mjModel_sensor_type = field _mjModel "sensor_type" (ptr int)

  (**  numeric data type (mjtDataType)          (nsensor x 1) *)
  let mjModel_sensor_datatype = field _mjModel "sensor_datatype" (ptr int)

  (**  required compute stage (mjtStage)        (nsensor x 1) *)
  let mjModel_sensor_needstage = field _mjModel "sensor_needstage" (ptr int)

  (**  type of sensorized object (mjtObj)       (nsensor x 1) *)
  let mjModel_sensor_objtype = field _mjModel "sensor_objtype" (ptr int)

  (**  id of sensorized object                  (nsensor x 1) *)
  let mjModel_sensor_objid = field _mjModel "sensor_objid" (ptr int)

  (**  number of scalar outputs                 (nsensor x 1) *)
  let mjModel_sensor_dim = field _mjModel "sensor_dim" (ptr int)

  (**  address in sensor array                  (nsensor x 1) *)
  let mjModel_sensor_adr = field _mjModel "sensor_adr" (ptr int)

  (**  cutoff for real and positive; 0: ignore  (nsensor x 1) *)
  let mjModel_sensor_cutoff = field _mjModel "sensor_cutoff" (ptr mjtNum)

  (**  noise standard deviation                 (nsensor x 1) *)
  let mjModel_sensor_noise = field _mjModel "sensor_noise" (ptr mjtNum)

  (**  user data                                (nsensor x nuser_sensor) *)
  let mjModel_sensor_user = field _mjModel "sensor_user" (ptr mjtNum)

  (**  address of field in numeric_data         (nnumeric x 1) *)
  let mjModel_numeric_adr = field _mjModel "numeric_adr" (ptr int)

  (**  size of numeric field                    (nnumeric x 1) *)
  let mjModel_numeric_size = field _mjModel "numeric_size" (ptr int)

  (**  array of all numeric fields              (nnumericdata x 1) *)
  let mjModel_numeric_data = field _mjModel "numeric_data" (ptr mjtNum)

  (**  address of text in text_data             (ntext x 1) *)
  let mjModel_text_adr = field _mjModel "text_adr" (ptr int)

  (**  size of text field (strlen+1)            (ntext x 1) *)
  let mjModel_text_size = field _mjModel "text_size" (ptr int)

  (**  array of all text fields (0-terminated)  (ntextdata x 1) *)
  let mjModel_text_data = field _mjModel "text_data" string

  (**  address of text in text_data             (ntuple x 1) *)
  let mjModel_tuple_adr = field _mjModel "tuple_adr" (ptr int)

  (**  number of objects in tuple               (ntuple x 1) *)
  let mjModel_tuple_size = field _mjModel "tuple_size" (ptr int)

  (**  array of object types in all tuples      (ntupledata x 1) *)
  let mjModel_tuple_objtype = field _mjModel "tuple_objtype" (ptr int)

  (**  array of object ids in all tuples        (ntupledata x 1) *)
  let mjModel_tuple_objid = field _mjModel "tuple_objid" (ptr int)

  (**  array of object params in all tuples     (ntupledata x 1) *)
  let mjModel_tuple_objprm = field _mjModel "tuple_objprm" (ptr mjtNum)

  (**  key time                                 (nkey x 1) *)
  let mjModel_key_time = field _mjModel "key_time" (ptr mjtNum)

  (**  key position                             (nkey x nq) *)
  let mjModel_key_qpos = field _mjModel "key_qpos" (ptr mjtNum)

  (**  key velocity                             (nkey x nv) *)
  let mjModel_key_qvel = field _mjModel "key_qvel" (ptr mjtNum)

  (**  key activation                           (nkey x na) *)
  let mjModel_key_act = field _mjModel "key_act" (ptr mjtNum)

  (**  key mocap position                       (nkey x 3*nmocap) *)
  let mjModel_key_mpos = field _mjModel "key_mpos" (ptr mjtNum)

  (**  key mocap quaternion                     (nkey x 4*nmocap) *)
  let mjModel_key_mquat = field _mjModel "key_mquat" (ptr mjtNum)

  (**  body name pointers                       (nbody x 1) *)
  let mjModel_name_bodyadr = field _mjModel "name_bodyadr" (ptr int)

  (**  joint name pointers                      (njnt x 1) *)
  let mjModel_name_jntadr = field _mjModel "name_jntadr" (ptr int)

  (**  geom name pointers                       (ngeom x 1) *)
  let mjModel_name_geomadr = field _mjModel "name_geomadr" (ptr int)

  (**  site name pointers                       (nsite x 1) *)
  let mjModel_name_siteadr = field _mjModel "name_siteadr" (ptr int)

  (**  camera name pointers                     (ncam x 1) *)
  let mjModel_name_camadr = field _mjModel "name_camadr" (ptr int)

  (**  light name pointers                      (nlight x 1) *)
  let mjModel_name_lightadr = field _mjModel "name_lightadr" (ptr int)

  (**  mesh name pointers                       (nmesh x 1) *)
  let mjModel_name_meshadr = field _mjModel "name_meshadr" (ptr int)

  (**  skin name pointers                       (nskin x 1) *)
  let mjModel_name_skinadr = field _mjModel "name_skinadr" (ptr int)

  (**  hfield name pointers                     (nhfield x 1) *)
  let mjModel_name_hfieldadr = field _mjModel "name_hfieldadr" (ptr int)

  (**  texture name pointers                    (ntex x 1) *)
  let mjModel_name_texadr = field _mjModel "name_texadr" (ptr int)

  (**  material name pointers                   (nmat x 1) *)
  let mjModel_name_matadr = field _mjModel "name_matadr" (ptr int)

  (**  geom pair name pointers                  (npair x 1) *)
  let mjModel_name_pairadr = field _mjModel "name_pairadr" (ptr int)

  (**  exclude name pointers                    (nexclude x 1) *)
  let mjModel_name_excludeadr = field _mjModel "name_excludeadr" (ptr int)

  (**  equality constraint name pointers        (neq x 1) *)
  let mjModel_name_eqadr = field _mjModel "name_eqadr" (ptr int)

  (**  tendon name pointers                     (ntendon x 1) *)
  let mjModel_name_tendonadr = field _mjModel "name_tendonadr" (ptr int)

  (**  actuator name pointers                   (nu x 1) *)
  let mjModel_name_actuatoradr = field _mjModel "name_actuatoradr" (ptr int)

  (**  sensor name pointers                     (nsensor x 1) *)
  let mjModel_name_sensoradr = field _mjModel "name_sensoradr" (ptr int)

  (**  numeric name pointers                    (nnumeric x 1) *)
  let mjModel_name_numericadr = field _mjModel "name_numericadr" (ptr int)

  (**  text name pointers                       (ntext x 1) *)
  let mjModel_name_textadr = field _mjModel "name_textadr" (ptr int)

  (**  tuple name pointers                      (ntuple x 1) *)
  let mjModel_name_tupleadr = field _mjModel "name_tupleadr" (ptr int)

  (**  keyframe name pointers                   (nkey x 1) *)
  let mjModel_name_keyadr = field _mjModel "name_keyadr" (ptr int)

  (**  names of all objects, 0-terminated       (nnames x 1) *)
  let mjModel_names = field _mjModel "names" string

  let () = seal _mjModel

  type mjModel = _mjModel

  let mjModel = _mjModel
  (* -------------------------------------------------------------------------------- *)
  (* ------------------------------ mjdata.h ---------------------------------------- *)
  (* -------------------------------------------------------------------------------- *)

  (**  warning types *)
  type mjtWarning =
    | MjWARN_INERTIA
    | MjWARN_CONTACTFULL
    | MjWARN_CNSTRFULL
    | MjWARN_VGEOMFULL
    | MjWARN_BADQPOS
    | MjWARN_BADQVEL
    | MjWARN_BADQACC
    | MjWARN_BADCTRL
    | MjNWARNING

  let mjWARN_INERTIA = constant "mjWARN_INERTIA" int64_t
  let mjWARN_CONTACTFULL = constant "mjWARN_CONTACTFULL" int64_t
  let mjWARN_CNSTRFULL = constant "mjWARN_CNSTRFULL" int64_t
  let mjWARN_VGEOMFULL = constant "mjWARN_VGEOMFULL" int64_t
  let mjWARN_BADQPOS = constant "mjWARN_BADQPOS" int64_t
  let mjWARN_BADQVEL = constant "mjWARN_BADQVEL" int64_t
  let mjWARN_BADQACC = constant "mjWARN_BADQACC" int64_t
  let mjWARN_BADCTRL = constant "mjWARN_BADCTRL" int64_t
  let mjNWARNING = constant "mjNWARNING" int64_t

  let mjtWarning =
    S.enum
      "mjtWarning"
      ~typedef:true
      [ MjWARN_INERTIA, mjWARN_INERTIA
      ; MjWARN_CONTACTFULL, mjWARN_CONTACTFULL
      ; MjWARN_CNSTRFULL, mjWARN_CNSTRFULL
      ; MjWARN_VGEOMFULL, mjWARN_VGEOMFULL
      ; MjWARN_BADQPOS, mjWARN_BADQPOS
      ; MjWARN_BADQVEL, mjWARN_BADQVEL
      ; MjWARN_BADQACC, mjWARN_BADQACC
      ; MjWARN_BADCTRL, mjWARN_BADCTRL
      ; MjNWARNING, mjNWARNING
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtWarning element data type enum")


  (**  *)
  type mjtTimer =
    | MjTIMER_STEP
    | MjTIMER_FORWARD
    | MjTIMER_INVERSE
    | MjTIMER_POSITION
    | MjTIMER_VELOCITY
    | MjTIMER_ACTUATION
    | MjTIMER_ACCELERATION
    | MjTIMER_CONSTRAINT
    | MjTIMER_POS_KINEMATICS
    | MjTIMER_POS_INERTIA
    | MjTIMER_POS_COLLISION
    | MjTIMER_POS_MAKE
    | MjTIMER_POS_PROJECT
    | MjNTIMER

  let mjTIMER_STEP = constant "mjTIMER_STEP" int64_t
  let mjTIMER_FORWARD = constant "mjTIMER_FORWARD" int64_t
  let mjTIMER_INVERSE = constant "mjTIMER_INVERSE" int64_t
  let mjTIMER_POSITION = constant "mjTIMER_POSITION" int64_t
  let mjTIMER_VELOCITY = constant "mjTIMER_VELOCITY" int64_t
  let mjTIMER_ACTUATION = constant "mjTIMER_ACTUATION" int64_t
  let mjTIMER_ACCELERATION = constant "mjTIMER_ACCELERATION" int64_t
  let mjTIMER_CONSTRAINT = constant "mjTIMER_CONSTRAINT" int64_t
  let mjTIMER_POS_KINEMATICS = constant "mjTIMER_POS_KINEMATICS" int64_t
  let mjTIMER_POS_INERTIA = constant "mjTIMER_POS_INERTIA" int64_t
  let mjTIMER_POS_COLLISION = constant "mjTIMER_POS_COLLISION" int64_t
  let mjTIMER_POS_MAKE = constant "mjTIMER_POS_MAKE" int64_t
  let mjTIMER_POS_PROJECT = constant "mjTIMER_POS_PROJECT" int64_t
  let mjNTIMER = constant "mjNTIMER" int64_t

  let mjtTimer =
    S.enum
      "mjtTimer"
      ~typedef:true
      [ MjTIMER_STEP, mjTIMER_STEP
      ; MjTIMER_FORWARD, mjTIMER_FORWARD
      ; MjTIMER_INVERSE, mjTIMER_INVERSE
      ; MjTIMER_POSITION, mjTIMER_POSITION
      ; MjTIMER_VELOCITY, mjTIMER_VELOCITY
      ; MjTIMER_ACTUATION, mjTIMER_ACTUATION
      ; MjTIMER_ACCELERATION, mjTIMER_ACCELERATION
      ; MjTIMER_CONSTRAINT, mjTIMER_CONSTRAINT
      ; MjTIMER_POS_KINEMATICS, mjTIMER_POS_KINEMATICS
      ; MjTIMER_POS_INERTIA, mjTIMER_POS_INERTIA
      ; MjTIMER_POS_COLLISION, mjTIMER_POS_COLLISION
      ; MjTIMER_POS_MAKE, mjTIMER_POS_MAKE
      ; MjTIMER_POS_PROJECT, mjTIMER_POS_PROJECT
      ; MjNTIMER, mjNTIMER
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtTimer element data type enum")


  type _mjContact

  let _mjContact : _mjContact structure typ = structure "_mjContact"

  (**  distance between nearest points; neg: penetration *)
  let mjContact_dist = field _mjContact "dist" mjtNum

  (**  position of contact point: midpoint between geoms *)
  let mjContact_pos = field _mjContact "pos" (ptr mjtNum)

  (**  normal is in [0-2] *)
  let mjContact_frame = field _mjContact "frame" (ptr mjtNum)

  (**  include if dist<includemargin=margin-gap *)
  let mjContact_includemargin = field _mjContact "includemargin" mjtNum

  (**  tangent1, 2, spin, roll1, 2 *)
  let mjContact_friction = field _mjContact "friction" (ptr mjtNum)

  (**  constraint solver reference *)
  let mjContact_solref = field _mjContact "solref" (ptr mjtNum)

  (**  constraint solver impedance *)
  let mjContact_solimp = field _mjContact "solimp" (ptr mjtNum)

  (**  friction of regularized cone, set by mj_makeConstraint *)
  let mjContact_mu = field _mjContact "mu" mjtNum

  (**  cone Hessian, set by mj_updateConstraint *)
  let mjContact_H = field _mjContact "H" (ptr mjtNum)

  (**  contact space dimensionality: 1, 3, 4 or 6 *)
  let mjContact_dim = field _mjContact "dim" int

  (**  id of geom 1 *)
  let mjContact_geom1 = field _mjContact "geom1" int

  (**  id of geom 2 *)
  let mjContact_geom2 = field _mjContact "geom2" int

  (**  0: include, 1: in gap, 2: fused, 3: equality, 4: no dofs *)
  let mjContact_exclude = field _mjContact "exclude" int

  (**  address in efc; -1: not included, -2-i: distance constraint i *)
  let mjContact_efc_address = field _mjContact "efc_address" int

  let () = seal _mjContact

  type mjContact = _mjContact

  let mjContact = _mjContact

  type _mjWarningStat

  let _mjWarningStat : _mjWarningStat structure typ = structure "_mjWarningStat"

  (**  info from last warning *)
  let mjWarningStat_lastinfo = field _mjWarningStat "lastinfo" int

  (**  how many times was warning raised *)
  let mjWarningStat_number = field _mjWarningStat "number" int

  let () = seal _mjWarningStat

  type mjWarningStat = _mjWarningStat

  let mjWarningStat = _mjWarningStat

  type _mjTimerStat

  let _mjTimerStat : _mjTimerStat structure typ = structure "_mjTimerStat"

  (**  cumulative duration *)
  let mjTimerStat_duration = field _mjTimerStat "duration" mjtNum

  (**  how many times was timer called *)
  let mjTimerStat_number = field _mjTimerStat "number" int

  let () = seal _mjTimerStat

  type mjTimerStat = _mjTimerStat

  let mjTimerStat = _mjTimerStat

  type _mjSolverStat

  let _mjSolverStat : _mjSolverStat structure typ = structure "_mjSolverStat"

  (**  cost reduction, scaled by 1/trace(M(qpos0)) *)
  let mjSolverStat_improvement = field _mjSolverStat "improvement" mjtNum

  (**  gradient norm (primal only, scaled) *)
  let mjSolverStat_gradient = field _mjSolverStat "gradient" mjtNum

  (**  slope in linesearch *)
  let mjSolverStat_lineslope = field _mjSolverStat "lineslope" mjtNum

  (**  number of active constraints *)
  let mjSolverStat_nactive = field _mjSolverStat "nactive" int

  (**  number of constraint state changes *)
  let mjSolverStat_nchange = field _mjSolverStat "nchange" int

  (**  number of cost evaluations in line search *)
  let mjSolverStat_neval = field _mjSolverStat "neval" int

  (**  number of Cholesky updates in line search *)
  let mjSolverStat_nupdate = field _mjSolverStat "nupdate" int

  let () = seal _mjSolverStat

  type mjSolverStat = _mjSolverStat

  let mjSolverStat = _mjSolverStat

  type _mjData

  let _mjData : _mjData structure typ = structure "_mjData"

  (**  number of mjtNums that can fit in stack *)
  let mjData_nstack = field _mjData "nstack" int

  (**  size of main buffer in bytes *)
  let mjData_nbuffer = field _mjData "nbuffer" int

  (**  first available mjtNum address in stack *)
  let mjData_pstack = field _mjData "pstack" int

  (**  maximum stack allocation *)
  let mjData_maxuse_stack = field _mjData "maxuse_stack" int

  (**  maximum number of contacts *)
  let mjData_maxuse_con = field _mjData "maxuse_con" int

  (**  maximum number of scalar constraints *)
  let mjData_maxuse_efc = field _mjData "maxuse_efc" int

  (**  warning statistics *)
  let mjData_warning = field _mjData "warning" (ptr mjWarningStat)

  (**  timer statistics *)
  let mjData_timer = field _mjData "timer" (ptr mjTimerStat)

  (**  solver statistics per iteration *)
  let mjData_solver = field _mjData "solver" (ptr mjSolverStat)

  (**  number of solver iterations *)
  let mjData_solver_iter = field _mjData "solver_iter" int

  (**  number of non-zeros in Hessian or efc_AR *)
  let mjData_solver_nnz = field _mjData "solver_nnz" int

  (**  forward-inverse comparison: qfrc, efc *)
  let mjData_solver_fwdinv = field _mjData "solver_fwdinv" (ptr mjtNum)

  (**  number of equality constraints *)
  let mjData_ne = field _mjData "ne" int

  (**  number of friction constraints *)
  let mjData_nf = field _mjData "nf" int

  (**  number of constraints *)
  let mjData_nefc = field _mjData "nefc" int

  (**  number of detected contacts *)
  let mjData_ncon = field _mjData "ncon" int

  (**  simulation time *)
  let mjData_time = field _mjData "time" mjtNum

  (**  potential, kinetic energy *)
  let mjData_energy = field _mjData "energy" (ptr mjtNum)

  (**  main buffer; all pointers point in it    (nbuffer bytes) *)
  let mjData_buffer = field _mjData "buffer" (ptr void)

  (**  stack buffer                             (nstack mjtNums) *)
  let mjData_stack = field _mjData "stack" (ptr mjtNum)

  (**  position                                 (nq x 1) *)
  let mjData_qpos = field _mjData "qpos" (ptr mjtNum)

  (**  velocity                                 (nv x 1) *)
  let mjData_qvel = field _mjData "qvel" (ptr mjtNum)

  (**  actuator activation                      (na x 1) *)
  let mjData_act = field _mjData "act" (ptr mjtNum)

  (**  acceleration used for warmstart          (nv x 1) *)
  let mjData_qacc_warmstart = field _mjData "qacc_warmstart" (ptr mjtNum)

  (**  control                                  (nu x 1) *)
  let mjData_ctrl = field _mjData "ctrl" (ptr mjtNum)

  (**  applied generalized force                (nv x 1) *)
  let mjData_qfrc_applied = field _mjData "qfrc_applied" (ptr mjtNum)

  (**  applied Cartesian force/torque           (nbody x 6) *)
  let mjData_xfrc_applied = field _mjData "xfrc_applied" (ptr mjtNum)

  (**  acceleration                             (nv x 1) *)
  let mjData_qacc = field _mjData "qacc" (ptr mjtNum)

  (**  time-derivative of actuator activation   (na x 1) *)
  let mjData_act_dot = field _mjData "act_dot" (ptr mjtNum)

  (**  positions of mocap bodies                (nmocap x 3) *)
  let mjData_mocap_pos = field _mjData "mocap_pos" (ptr mjtNum)

  (**  orientations of mocap bodies             (nmocap x 4) *)
  let mjData_mocap_quat = field _mjData "mocap_quat" (ptr mjtNum)

  (**  user data, not touched by engine         (nuserdata x 1) *)
  let mjData_userdata = field _mjData "userdata" (ptr mjtNum)

  (**  sensor data array                        (nsensordata x 1) *)
  let mjData_sensordata = field _mjData "sensordata" (ptr mjtNum)

  (**  Cartesian position of body frame         (nbody x 3) *)
  let mjData_xpos = field _mjData "xpos" (ptr mjtNum)

  (**  Cartesian orientation of body frame      (nbody x 4) *)
  let mjData_xquat = field _mjData "xquat" (ptr mjtNum)

  (**  Cartesian orientation of body frame      (nbody x 9) *)
  let mjData_xmat = field _mjData "xmat" (ptr mjtNum)

  (**  Cartesian position of body com           (nbody x 3) *)
  let mjData_xipos = field _mjData "xipos" (ptr mjtNum)

  (**  Cartesian orientation of body inertia    (nbody x 9) *)
  let mjData_ximat = field _mjData "ximat" (ptr mjtNum)

  (**  Cartesian position of joint anchor       (njnt x 3) *)
  let mjData_xanchor = field _mjData "xanchor" (ptr mjtNum)

  (**  Cartesian joint axis                     (njnt x 3) *)
  let mjData_xaxis = field _mjData "xaxis" (ptr mjtNum)

  (**  Cartesian geom position                  (ngeom x 3) *)
  let mjData_geom_xpos = field _mjData "geom_xpos" (ptr mjtNum)

  (**  Cartesian geom orientation               (ngeom x 9) *)
  let mjData_geom_xmat = field _mjData "geom_xmat" (ptr mjtNum)

  (**  Cartesian site position                  (nsite x 3) *)
  let mjData_site_xpos = field _mjData "site_xpos" (ptr mjtNum)

  (**  Cartesian site orientation               (nsite x 9) *)
  let mjData_site_xmat = field _mjData "site_xmat" (ptr mjtNum)

  (**  Cartesian camera position                (ncam x 3) *)
  let mjData_cam_xpos = field _mjData "cam_xpos" (ptr mjtNum)

  (**  Cartesian camera orientation             (ncam x 9) *)
  let mjData_cam_xmat = field _mjData "cam_xmat" (ptr mjtNum)

  (**  Cartesian light position                 (nlight x 3) *)
  let mjData_light_xpos = field _mjData "light_xpos" (ptr mjtNum)

  (**  Cartesian light direction                (nlight x 3) *)
  let mjData_light_xdir = field _mjData "light_xdir" (ptr mjtNum)

  (**  center of mass of each subtree           (nbody x 3) *)
  let mjData_subtree_com = field _mjData "subtree_com" (ptr mjtNum)

  (**  com-based motion axis of each dof        (nv x 6) *)
  let mjData_cdof = field _mjData "cdof" (ptr mjtNum)

  (**  com-based body inertia and mass          (nbody x 10) *)
  let mjData_cinert = field _mjData "cinert" (ptr mjtNum)

  (**  start address of tendon's path           (ntendon x 1) *)
  let mjData_ten_wrapadr = field _mjData "ten_wrapadr" (ptr int)

  (**  number of wrap points in path            (ntendon x 1) *)
  let mjData_ten_wrapnum = field _mjData "ten_wrapnum" (ptr int)

  (**  number of non-zeros in Jacobian row      (ntendon x 1) *)
  let mjData_ten_J_rownnz = field _mjData "ten_J_rownnz" (ptr int)

  (**  row start address in colind array        (ntendon x 1) *)
  let mjData_ten_J_rowadr = field _mjData "ten_J_rowadr" (ptr int)

  (**  column indices in sparse Jacobian        (ntendon x nv) *)
  let mjData_ten_J_colind = field _mjData "ten_J_colind" (ptr int)

  (**  tendon lengths                           (ntendon x 1) *)
  let mjData_ten_length = field _mjData "ten_length" (ptr mjtNum)

  (**  tendon Jacobian                          (ntendon x nv) *)
  let mjData_ten_J = field _mjData "ten_J" (ptr mjtNum)

  (**  geom id; -1: site; -2: pulley            (nwrap*2 x 1) *)
  let mjData_wrap_obj = field _mjData "wrap_obj" (ptr int)

  (**  Cartesian 3D points in all path          (nwrap*2 x 3) *)
  let mjData_wrap_xpos = field _mjData "wrap_xpos" (ptr mjtNum)

  (**  actuator lengths                         (nu x 1) *)
  let mjData_actuator_length = field _mjData "actuator_length" (ptr mjtNum)

  (**  actuator moments                         (nu x nv) *)
  let mjData_actuator_moment = field _mjData "actuator_moment" (ptr mjtNum)

  (**  com-based composite inertia and mass     (nbody x 10) *)
  let mjData_crb = field _mjData "crb" (ptr mjtNum)

  (**  total inertia                            (nM x 1) *)
  let mjData_qM = field _mjData "qM" (ptr mjtNum)

  (**  L'*D*L factorization of M                (nM x 1) *)
  let mjData_qLD = field _mjData "qLD" (ptr mjtNum)

  (**  1/diag(D)                                (nv x 1) *)
  let mjData_qLDiagInv = field _mjData "qLDiagInv" (ptr mjtNum)

  (**  1/sqrt(diag(D))                          (nv x 1) *)
  let mjData_qLDiagSqrtInv = field _mjData "qLDiagSqrtInv" (ptr mjtNum)

  (**  list of all detected contacts            (nconmax x 1) *)
  let mjData_contact = field _mjData "contact" (ptr mjContact)

  (**  constraint type (mjtConstraint)          (njmax x 1) *)
  let mjData_efc_type = field _mjData "efc_type" (ptr int)

  (**  id of object of specified type           (njmax x 1) *)
  let mjData_efc_id = field _mjData "efc_id" (ptr int)

  (**  number of non-zeros in Jacobian row      (njmax x 1) *)
  let mjData_efc_J_rownnz = field _mjData "efc_J_rownnz" (ptr int)

  (**  row start address in colind array        (njmax x 1) *)
  let mjData_efc_J_rowadr = field _mjData "efc_J_rowadr" (ptr int)

  (**  number of subsequent rows in supernode   (njmax x 1) *)
  let mjData_efc_J_rowsuper = field _mjData "efc_J_rowsuper" (ptr int)

  (**  column indices in Jacobian               (njmax x nv) *)
  let mjData_efc_J_colind = field _mjData "efc_J_colind" (ptr int)

  (**  number of non-zeros in Jacobian row    T (nv x 1) *)
  let mjData_efc_JT_rownnz = field _mjData "efc_JT_rownnz" (ptr int)

  (**  row start address in colind array      T (nv x 1) *)
  let mjData_efc_JT_rowadr = field _mjData "efc_JT_rowadr" (ptr int)

  (**  number of subsequent rows in supernode T (nv x 1) *)
  let mjData_efc_JT_rowsuper = field _mjData "efc_JT_rowsuper" (ptr int)

  (**  column indices in Jacobian             T (nv x njmax) *)
  let mjData_efc_JT_colind = field _mjData "efc_JT_colind" (ptr int)

  (**  constraint Jacobian                      (njmax x nv) *)
  let mjData_efc_J = field _mjData "efc_J" (ptr mjtNum)

  (**  constraint Jacobian transposed           (nv x njmax) *)
  let mjData_efc_JT = field _mjData "efc_JT" (ptr mjtNum)

  (**  constraint position (equality, contact)  (njmax x 1) *)
  let mjData_efc_pos = field _mjData "efc_pos" (ptr mjtNum)

  (**  inclusion margin (contact)               (njmax x 1) *)
  let mjData_efc_margin = field _mjData "efc_margin" (ptr mjtNum)

  (**  frictionloss (friction)                  (njmax x 1) *)
  let mjData_efc_frictionloss = field _mjData "efc_frictionloss" (ptr mjtNum)

  (**  approximation to diagonal of A           (njmax x 1) *)
  let mjData_efc_diagApprox = field _mjData "efc_diagApprox" (ptr mjtNum)

  (**  stiffness, damping, impedance, imp'      (njmax x 4) *)
  let mjData_efc_KBIP = field _mjData "efc_KBIP" (ptr mjtNum)

  (**  constraint mass                          (njmax x 1) *)
  let mjData_efc_D = field _mjData "efc_D" (ptr mjtNum)

  (**  inverse constraint mass                  (njmax x 1) *)
  let mjData_efc_R = field _mjData "efc_R" (ptr mjtNum)

  (**  number of non-zeros in AR                (njmax x 1) *)
  let mjData_efc_AR_rownnz = field _mjData "efc_AR_rownnz" (ptr int)

  (**  row start address in colind array        (njmax x 1) *)
  let mjData_efc_AR_rowadr = field _mjData "efc_AR_rowadr" (ptr int)

  (**  column indices in sparse AR              (njmax x njmax) *)
  let mjData_efc_AR_colind = field _mjData "efc_AR_colind" (ptr int)

  (**  J*inv(M)*J' + R                          (njmax x njmax) *)
  let mjData_efc_AR = field _mjData "efc_AR" (ptr mjtNum)

  (**  tendon velocities                        (ntendon x 1) *)
  let mjData_ten_velocity = field _mjData "ten_velocity" (ptr mjtNum)

  (**  actuator velocities                      (nu x 1) *)
  let mjData_actuator_velocity = field _mjData "actuator_velocity" (ptr mjtNum)

  (**  com-based velocity [3D rot; 3D tran]     (nbody x 6) *)
  let mjData_cvel = field _mjData "cvel" (ptr mjtNum)

  (**  time-derivative of cdof                  (nv x 6) *)
  let mjData_cdof_dot = field _mjData "cdof_dot" (ptr mjtNum)

  (**  C(qpos,qvel)                             (nv x 1) *)
  let mjData_qfrc_bias = field _mjData "qfrc_bias" (ptr mjtNum)

  (**  passive force                            (nv x 1) *)
  let mjData_qfrc_passive = field _mjData "qfrc_passive" (ptr mjtNum)

  (**  velocity in constraint space: J*qvel     (njmax x 1) *)
  let mjData_efc_vel = field _mjData "efc_vel" (ptr mjtNum)

  (**  reference pseudo-acceleration            (njmax x 1) *)
  let mjData_efc_aref = field _mjData "efc_aref" (ptr mjtNum)

  (**  linear velocity of subtree com           (nbody x 3) *)
  let mjData_subtree_linvel = field _mjData "subtree_linvel" (ptr mjtNum)

  (**  angular momentum about subtree com       (nbody x 3) *)
  let mjData_subtree_angmom = field _mjData "subtree_angmom" (ptr mjtNum)

  (**  actuator force in actuation space        (nu x 1) *)
  let mjData_actuator_force = field _mjData "actuator_force" (ptr mjtNum)

  (**  actuator force                           (nv x 1) *)
  let mjData_qfrc_actuator = field _mjData "qfrc_actuator" (ptr mjtNum)

  (**  net unconstrained force                  (nv x 1) *)
  let mjData_qfrc_unc = field _mjData "qfrc_unc" (ptr mjtNum)

  (**  unconstrained acceleration               (nv x 1) *)
  let mjData_qacc_unc = field _mjData "qacc_unc" (ptr mjtNum)

  (**  linear cost term: J*qacc_unc - aref      (njmax x 1) *)
  let mjData_efc_b = field _mjData "efc_b" (ptr mjtNum)

  (**  constraint force in constraint space     (njmax x 1) *)
  let mjData_efc_force = field _mjData "efc_force" (ptr mjtNum)

  (**  constraint state (mjtConstraintState)    (njmax x 1) *)
  let mjData_efc_state = field _mjData "efc_state" (ptr int)

  (**  constraint force                         (nv x 1) *)
  let mjData_qfrc_constraint = field _mjData "qfrc_constraint" (ptr mjtNum)

  (**  net external force; should equal:        (nv x 1) *)
  let mjData_qfrc_inverse = field _mjData "qfrc_inverse" (ptr mjtNum)

  (**  com-based acceleration                   (nbody x 6) *)
  let mjData_cacc = field _mjData "cacc" (ptr mjtNum)

  (**  com-based interaction force with parent  (nbody x 6) *)
  let mjData_cfrc_int = field _mjData "cfrc_int" (ptr mjtNum)

  (**  com-based external force on body         (nbody x 6) *)
  let mjData_cfrc_ext = field _mjData "cfrc_ext" (ptr mjtNum)

  let () = seal _mjData

  type mjData = _mjData

  let mjData = _mjData
  (* -------------------------------------------------------------------------------- *)
  (* ------------------------------ mjvisualize.h ----------------------------------- *)
  (* -------------------------------------------------------------------------------- *)

  (**  bitflags for mjvGeom category *)
  type mjtCatBit =
    | MjCAT_STATIC
    | MjCAT_DYNAMIC
    | MjCAT_DECOR
    | MjCAT_ALL

  let mjCAT_STATIC = constant "mjCAT_STATIC" int64_t
  let mjCAT_DYNAMIC = constant "mjCAT_DYNAMIC" int64_t
  let mjCAT_DECOR = constant "mjCAT_DECOR" int64_t
  let mjCAT_ALL = constant "mjCAT_ALL" int64_t

  let mjtCatBit =
    S.enum
      "mjtCatBit"
      ~typedef:true
      [ MjCAT_STATIC, mjCAT_STATIC
      ; MjCAT_DYNAMIC, mjCAT_DYNAMIC
      ; MjCAT_DECOR, mjCAT_DECOR
      ; MjCAT_ALL, mjCAT_ALL
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtCatBit element data type enum")


  (**  mouse interaction mode *)
  type mjtMouse =
    | MjMOUSE_NONE
    | MjMOUSE_ROTATE_V
    | MjMOUSE_ROTATE_H
    | MjMOUSE_MOVE_V
    | MjMOUSE_MOVE_H
    | MjMOUSE_ZOOM
    | MjMOUSE_SELECT

  let mjMOUSE_NONE = constant "mjMOUSE_NONE" int64_t
  let mjMOUSE_ROTATE_V = constant "mjMOUSE_ROTATE_V" int64_t
  let mjMOUSE_ROTATE_H = constant "mjMOUSE_ROTATE_H" int64_t
  let mjMOUSE_MOVE_V = constant "mjMOUSE_MOVE_V" int64_t
  let mjMOUSE_MOVE_H = constant "mjMOUSE_MOVE_H" int64_t
  let mjMOUSE_ZOOM = constant "mjMOUSE_ZOOM" int64_t
  let mjMOUSE_SELECT = constant "mjMOUSE_SELECT" int64_t

  let mjtMouse =
    S.enum
      "mjtMouse"
      ~typedef:true
      [ MjMOUSE_NONE, mjMOUSE_NONE
      ; MjMOUSE_ROTATE_V, mjMOUSE_ROTATE_V
      ; MjMOUSE_ROTATE_H, mjMOUSE_ROTATE_H
      ; MjMOUSE_MOVE_V, mjMOUSE_MOVE_V
      ; MjMOUSE_MOVE_H, mjMOUSE_MOVE_H
      ; MjMOUSE_ZOOM, mjMOUSE_ZOOM
      ; MjMOUSE_SELECT, mjMOUSE_SELECT
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtMouse element data type enum")


  (**  mouse perturbations *)
  type mjtPertBit =
    | MjPERT_TRANSLATE
    | MjPERT_ROTATE

  let mjPERT_TRANSLATE = constant "mjPERT_TRANSLATE" int64_t
  let mjPERT_ROTATE = constant "mjPERT_ROTATE" int64_t

  let mjtPertBit =
    S.enum
      "mjtPertBit"
      ~typedef:true
      [ MjPERT_TRANSLATE, mjPERT_TRANSLATE; MjPERT_ROTATE, mjPERT_ROTATE ]
      ~unexpected:(fun _ -> failwith "unexpected mjtPertBit element data type enum")


  (**  abstract camera type *)
  type mjtCamera =
    | MjCAMERA_FREE
    | MjCAMERA_TRACKING
    | MjCAMERA_FIXED
    | MjCAMERA_USER

  let mjCAMERA_FREE = constant "mjCAMERA_FREE" int64_t
  let mjCAMERA_TRACKING = constant "mjCAMERA_TRACKING" int64_t
  let mjCAMERA_FIXED = constant "mjCAMERA_FIXED" int64_t
  let mjCAMERA_USER = constant "mjCAMERA_USER" int64_t

  let mjtCamera =
    S.enum
      "mjtCamera"
      ~typedef:true
      [ MjCAMERA_FREE, mjCAMERA_FREE
      ; MjCAMERA_TRACKING, mjCAMERA_TRACKING
      ; MjCAMERA_FIXED, mjCAMERA_FIXED
      ; MjCAMERA_USER, mjCAMERA_USER
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtCamera element data type enum")


  (**  object labeling *)
  type mjtLabel =
    | MjLABEL_NONE
    | MjLABEL_BODY
    | MjLABEL_JOINT
    | MjLABEL_GEOM
    | MjLABEL_SITE
    | MjLABEL_CAMERA
    | MjLABEL_LIGHT
    | MjLABEL_TENDON
    | MjLABEL_ACTUATOR
    | MjLABEL_CONSTRAINT
    | MjLABEL_SKIN
    | MjLABEL_SELECTION
    | MjLABEL_SELPNT
    | MjLABEL_CONTACTFORCE
    | MjNLABEL

  let mjLABEL_NONE = constant "mjLABEL_NONE" int64_t
  let mjLABEL_BODY = constant "mjLABEL_BODY" int64_t
  let mjLABEL_JOINT = constant "mjLABEL_JOINT" int64_t
  let mjLABEL_GEOM = constant "mjLABEL_GEOM" int64_t
  let mjLABEL_SITE = constant "mjLABEL_SITE" int64_t
  let mjLABEL_CAMERA = constant "mjLABEL_CAMERA" int64_t
  let mjLABEL_LIGHT = constant "mjLABEL_LIGHT" int64_t
  let mjLABEL_TENDON = constant "mjLABEL_TENDON" int64_t
  let mjLABEL_ACTUATOR = constant "mjLABEL_ACTUATOR" int64_t
  let mjLABEL_CONSTRAINT = constant "mjLABEL_CONSTRAINT" int64_t
  let mjLABEL_SKIN = constant "mjLABEL_SKIN" int64_t
  let mjLABEL_SELECTION = constant "mjLABEL_SELECTION" int64_t
  let mjLABEL_SELPNT = constant "mjLABEL_SELPNT" int64_t
  let mjLABEL_CONTACTFORCE = constant "mjLABEL_CONTACTFORCE" int64_t
  let mjNLABEL = constant "mjNLABEL" int64_t

  let mjtLabel =
    S.enum
      "mjtLabel"
      ~typedef:true
      [ MjLABEL_NONE, mjLABEL_NONE
      ; MjLABEL_BODY, mjLABEL_BODY
      ; MjLABEL_JOINT, mjLABEL_JOINT
      ; MjLABEL_GEOM, mjLABEL_GEOM
      ; MjLABEL_SITE, mjLABEL_SITE
      ; MjLABEL_CAMERA, mjLABEL_CAMERA
      ; MjLABEL_LIGHT, mjLABEL_LIGHT
      ; MjLABEL_TENDON, mjLABEL_TENDON
      ; MjLABEL_ACTUATOR, mjLABEL_ACTUATOR
      ; MjLABEL_CONSTRAINT, mjLABEL_CONSTRAINT
      ; MjLABEL_SKIN, mjLABEL_SKIN
      ; MjLABEL_SELECTION, mjLABEL_SELECTION
      ; MjLABEL_SELPNT, mjLABEL_SELPNT
      ; MjLABEL_CONTACTFORCE, mjLABEL_CONTACTFORCE
      ; MjNLABEL, mjNLABEL
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtLabel element data type enum")


  (**  frame visualization *)
  type mjtFrame =
    | MjFRAME_NONE
    | MjFRAME_BODY
    | MjFRAME_GEOM
    | MjFRAME_SITE
    | MjFRAME_CAMERA
    | MjFRAME_LIGHT
    | MjFRAME_WORLD
    | MjNFRAME

  let mjFRAME_NONE = constant "mjFRAME_NONE" int64_t
  let mjFRAME_BODY = constant "mjFRAME_BODY" int64_t
  let mjFRAME_GEOM = constant "mjFRAME_GEOM" int64_t
  let mjFRAME_SITE = constant "mjFRAME_SITE" int64_t
  let mjFRAME_CAMERA = constant "mjFRAME_CAMERA" int64_t
  let mjFRAME_LIGHT = constant "mjFRAME_LIGHT" int64_t
  let mjFRAME_WORLD = constant "mjFRAME_WORLD" int64_t
  let mjNFRAME = constant "mjNFRAME" int64_t

  let mjtFrame =
    S.enum
      "mjtFrame"
      ~typedef:true
      [ MjFRAME_NONE, mjFRAME_NONE
      ; MjFRAME_BODY, mjFRAME_BODY
      ; MjFRAME_GEOM, mjFRAME_GEOM
      ; MjFRAME_SITE, mjFRAME_SITE
      ; MjFRAME_CAMERA, mjFRAME_CAMERA
      ; MjFRAME_LIGHT, mjFRAME_LIGHT
      ; MjFRAME_WORLD, mjFRAME_WORLD
      ; MjNFRAME, mjNFRAME
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtFrame element data type enum")


  (**  flags enabling model element visualization *)
  type mjtVisFlag =
    | MjVIS_CONVEXHULL
    | MjVIS_TEXTURE
    | MjVIS_JOINT
    | MjVIS_ACTUATOR
    | MjVIS_CAMERA
    | MjVIS_LIGHT
    | MjVIS_TENDON
    | MjVIS_RANGEFINDER
    | MjVIS_CONSTRAINT
    | MjVIS_INERTIA
    | MjVIS_SCLINERTIA
    | MjVIS_PERTFORCE
    | MjVIS_PERTOBJ
    | MjVIS_CONTACTPOINT
    | MjVIS_CONTACTFORCE
    | MjVIS_CONTACTSPLIT
    | MjVIS_TRANSPARENT
    | MjVIS_AUTOCONNECT
    | MjVIS_COM
    | MjVIS_SELECT
    | MjVIS_STATIC
    | MjVIS_SKIN
    | MjNVISFLAG

  let mjVIS_CONVEXHULL = constant "mjVIS_CONVEXHULL" int64_t
  let mjVIS_TEXTURE = constant "mjVIS_TEXTURE" int64_t
  let mjVIS_JOINT = constant "mjVIS_JOINT" int64_t
  let mjVIS_ACTUATOR = constant "mjVIS_ACTUATOR" int64_t
  let mjVIS_CAMERA = constant "mjVIS_CAMERA" int64_t
  let mjVIS_LIGHT = constant "mjVIS_LIGHT" int64_t
  let mjVIS_TENDON = constant "mjVIS_TENDON" int64_t
  let mjVIS_RANGEFINDER = constant "mjVIS_RANGEFINDER" int64_t
  let mjVIS_CONSTRAINT = constant "mjVIS_CONSTRAINT" int64_t
  let mjVIS_INERTIA = constant "mjVIS_INERTIA" int64_t
  let mjVIS_SCLINERTIA = constant "mjVIS_SCLINERTIA" int64_t
  let mjVIS_PERTFORCE = constant "mjVIS_PERTFORCE" int64_t
  let mjVIS_PERTOBJ = constant "mjVIS_PERTOBJ" int64_t
  let mjVIS_CONTACTPOINT = constant "mjVIS_CONTACTPOINT" int64_t
  let mjVIS_CONTACTFORCE = constant "mjVIS_CONTACTFORCE" int64_t
  let mjVIS_CONTACTSPLIT = constant "mjVIS_CONTACTSPLIT" int64_t
  let mjVIS_TRANSPARENT = constant "mjVIS_TRANSPARENT" int64_t
  let mjVIS_AUTOCONNECT = constant "mjVIS_AUTOCONNECT" int64_t
  let mjVIS_COM = constant "mjVIS_COM" int64_t
  let mjVIS_SELECT = constant "mjVIS_SELECT" int64_t
  let mjVIS_STATIC = constant "mjVIS_STATIC" int64_t
  let mjVIS_SKIN = constant "mjVIS_SKIN" int64_t
  let mjNVISFLAG = constant "mjNVISFLAG" int64_t

  let mjtVisFlag =
    S.enum
      "mjtVisFlag"
      ~typedef:true
      [ MjVIS_CONVEXHULL, mjVIS_CONVEXHULL
      ; MjVIS_TEXTURE, mjVIS_TEXTURE
      ; MjVIS_JOINT, mjVIS_JOINT
      ; MjVIS_ACTUATOR, mjVIS_ACTUATOR
      ; MjVIS_CAMERA, mjVIS_CAMERA
      ; MjVIS_LIGHT, mjVIS_LIGHT
      ; MjVIS_TENDON, mjVIS_TENDON
      ; MjVIS_RANGEFINDER, mjVIS_RANGEFINDER
      ; MjVIS_CONSTRAINT, mjVIS_CONSTRAINT
      ; MjVIS_INERTIA, mjVIS_INERTIA
      ; MjVIS_SCLINERTIA, mjVIS_SCLINERTIA
      ; MjVIS_PERTFORCE, mjVIS_PERTFORCE
      ; MjVIS_PERTOBJ, mjVIS_PERTOBJ
      ; MjVIS_CONTACTPOINT, mjVIS_CONTACTPOINT
      ; MjVIS_CONTACTFORCE, mjVIS_CONTACTFORCE
      ; MjVIS_CONTACTSPLIT, mjVIS_CONTACTSPLIT
      ; MjVIS_TRANSPARENT, mjVIS_TRANSPARENT
      ; MjVIS_AUTOCONNECT, mjVIS_AUTOCONNECT
      ; MjVIS_COM, mjVIS_COM
      ; MjVIS_SELECT, mjVIS_SELECT
      ; MjVIS_STATIC, mjVIS_STATIC
      ; MjVIS_SKIN, mjVIS_SKIN
      ; MjNVISFLAG, mjNVISFLAG
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtVisFlag element data type enum")


  (**  flags enabling rendering effects *)
  type mjtRndFlag =
    | MjRND_SHADOW
    | MjRND_WIREFRAME
    | MjRND_REFLECTION
    | MjRND_ADDITIVE
    | MjRND_SKYBOX
    | MjRND_FOG
    | MjRND_HAZE
    | MjRND_SEGMENT
    | MjRND_IDCOLOR
    | MjNRNDFLAG

  let mjRND_SHADOW = constant "mjRND_SHADOW" int64_t
  let mjRND_WIREFRAME = constant "mjRND_WIREFRAME" int64_t
  let mjRND_REFLECTION = constant "mjRND_REFLECTION" int64_t
  let mjRND_ADDITIVE = constant "mjRND_ADDITIVE" int64_t
  let mjRND_SKYBOX = constant "mjRND_SKYBOX" int64_t
  let mjRND_FOG = constant "mjRND_FOG" int64_t
  let mjRND_HAZE = constant "mjRND_HAZE" int64_t
  let mjRND_SEGMENT = constant "mjRND_SEGMENT" int64_t
  let mjRND_IDCOLOR = constant "mjRND_IDCOLOR" int64_t
  let mjNRNDFLAG = constant "mjNRNDFLAG" int64_t

  let mjtRndFlag =
    S.enum
      "mjtRndFlag"
      ~typedef:true
      [ MjRND_SHADOW, mjRND_SHADOW
      ; MjRND_WIREFRAME, mjRND_WIREFRAME
      ; MjRND_REFLECTION, mjRND_REFLECTION
      ; MjRND_ADDITIVE, mjRND_ADDITIVE
      ; MjRND_SKYBOX, mjRND_SKYBOX
      ; MjRND_FOG, mjRND_FOG
      ; MjRND_HAZE, mjRND_HAZE
      ; MjRND_SEGMENT, mjRND_SEGMENT
      ; MjRND_IDCOLOR, mjRND_IDCOLOR
      ; MjNRNDFLAG, mjNRNDFLAG
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtRndFlag element data type enum")


  (**  type of stereo rendering *)
  type mjtStereo =
    | MjSTEREO_NONE
    | MjSTEREO_QUADBUFFERED
    | MjSTEREO_SIDEBYSIDE

  let mjSTEREO_NONE = constant "mjSTEREO_NONE" int64_t
  let mjSTEREO_QUADBUFFERED = constant "mjSTEREO_QUADBUFFERED" int64_t
  let mjSTEREO_SIDEBYSIDE = constant "mjSTEREO_SIDEBYSIDE" int64_t

  let mjtStereo =
    S.enum
      "mjtStereo"
      ~typedef:true
      [ MjSTEREO_NONE, mjSTEREO_NONE
      ; MjSTEREO_QUADBUFFERED, mjSTEREO_QUADBUFFERED
      ; MjSTEREO_SIDEBYSIDE, mjSTEREO_SIDEBYSIDE
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtStereo element data type enum")


  type _mjvPerturb

  let _mjvPerturb : _mjvPerturb structure typ = structure "_mjvPerturb"

  (**  selected body id; non-positive: none *)
  let mjvPerturb_select = field _mjvPerturb "select" int

  (**  selected skin id; negative: none *)
  let mjvPerturb_skinselect = field _mjvPerturb "skinselect" int

  (**  perturbation bitmask (mjtPertBit) *)
  let mjvPerturb_active = field _mjvPerturb "active" int

  (**  secondary perturbation bitmask (mjtPertBit) *)
  let mjvPerturb_active2 = field _mjvPerturb "active2" int

  (**  desired position for selected object *)
  let mjvPerturb_refpos = field _mjvPerturb "refpos" (ptr mjtNum)

  (**  desired orientation for selected object *)
  let mjvPerturb_refquat = field _mjvPerturb "refquat" (ptr mjtNum)

  (**  selection point in object coordinates *)
  let mjvPerturb_localpos = field _mjvPerturb "localpos" (ptr mjtNum)

  (**  relative mouse motion-to-space scaling (set by initPerturb) *)
  let mjvPerturb_scale = field _mjvPerturb "scale" mjtNum

  let () = seal _mjvPerturb

  type mjvPerturb = _mjvPerturb

  let mjvPerturb = _mjvPerturb

  type _mjvCamera

  let _mjvCamera : _mjvCamera structure typ = structure "_mjvCamera"

  (**  camera type (mjtCamera) *)
  let mjvCamera_type = field _mjvCamera "type" int

  (**  fixed camera id *)
  let mjvCamera_fixedcamid = field _mjvCamera "fixedcamid" int

  (**  body id to track *)
  let mjvCamera_trackbodyid = field _mjvCamera "trackbodyid" int

  (**  lookat point *)
  let mjvCamera_lookat = field _mjvCamera "lookat" (ptr mjtNum)

  (**  distance to lookat point or tracked body *)
  let mjvCamera_distance = field _mjvCamera "distance" mjtNum

  (**  camera azimuth (deg) *)
  let mjvCamera_azimuth = field _mjvCamera "azimuth" mjtNum

  (**  camera elevation (deg) *)
  let mjvCamera_elevation = field _mjvCamera "elevation" mjtNum

  let () = seal _mjvCamera

  type mjvCamera = _mjvCamera

  let mjvCamera = _mjvCamera

  type _mjvGLCamera

  let _mjvGLCamera : _mjvGLCamera structure typ = structure "_mjvGLCamera"

  (**  position *)
  let mjvGLCamera_pos = field _mjvGLCamera "pos" (ptr float)

  (**  forward direction *)
  let mjvGLCamera_forward = field _mjvGLCamera "forward" (ptr float)

  (**  up direction *)
  let mjvGLCamera_up = field _mjvGLCamera "up" (ptr float)

  (**  hor. center (left,right set to match aspect) *)
  let mjvGLCamera_frustum_center = field _mjvGLCamera "frustum_center" float

  (**  bottom *)
  let mjvGLCamera_frustum_bottom = field _mjvGLCamera "frustum_bottom" float

  (**  top *)
  let mjvGLCamera_frustum_top = field _mjvGLCamera "frustum_top" float

  (**  near *)
  let mjvGLCamera_frustum_near = field _mjvGLCamera "frustum_near" float

  (**  far *)
  let mjvGLCamera_frustum_far = field _mjvGLCamera "frustum_far" float

  let () = seal _mjvGLCamera

  type mjvGLCamera = _mjvGLCamera

  let mjvGLCamera = _mjvGLCamera

  type _mjvGeom

  let _mjvGeom : _mjvGeom structure typ = structure "_mjvGeom"

  (**  geom type (mjtGeom) *)
  let mjvGeom_type = field _mjvGeom "type" int

  (**  mesh, hfield or plane id; -1: none *)
  let mjvGeom_dataid = field _mjvGeom "dataid" int

  (**  mujoco object type; mjOBJ_UNKNOWN for decor *)
  let mjvGeom_objtype = field _mjvGeom "objtype" int

  (**  mujoco object id; -1 for decor *)
  let mjvGeom_objid = field _mjvGeom "objid" int

  (**  visual category *)
  let mjvGeom_category = field _mjvGeom "category" int

  (**  texture id; -1: no texture *)
  let mjvGeom_texid = field _mjvGeom "texid" int

  (**  uniform cube mapping *)
  let mjvGeom_texuniform = field _mjvGeom "texuniform" int

  (**  mesh geom has texture coordinates *)
  let mjvGeom_texcoord = field _mjvGeom "texcoord" int

  (**  segmentation id; -1: not shown *)
  let mjvGeom_segid = field _mjvGeom "segid" int

  (**  texture repetition for 2D mapping *)
  let mjvGeom_texrepeat = field _mjvGeom "texrepeat" (ptr float)

  (**  size parameters *)
  let mjvGeom_size = field _mjvGeom "size" (ptr float)

  (**  Cartesian position *)
  let mjvGeom_pos = field _mjvGeom "pos" (ptr float)

  (**  Cartesian orientation *)
  let mjvGeom_mat = field _mjvGeom "mat" (ptr float)

  (**  color and transparency *)
  let mjvGeom_rgba = field _mjvGeom "rgba" (ptr float)

  (**  emission coef *)
  let mjvGeom_emission = field _mjvGeom "emission" float

  (**  specular coef *)
  let mjvGeom_specular = field _mjvGeom "specular" float

  (**  shininess coef *)
  let mjvGeom_shininess = field _mjvGeom "shininess" float

  (**  reflectance coef *)
  let mjvGeom_reflectance = field _mjvGeom "reflectance" float

  (**  text label *)
  let mjvGeom_label = field _mjvGeom "label" string

  (**  distance to camera (used by sorter) *)
  let mjvGeom_camdist = field _mjvGeom "camdist" float

  (**  geom rbound from model, 0 if not model geom *)
  let mjvGeom_modelrbound = field _mjvGeom "modelrbound" float

  (**  treat geom as transparent *)
  let mjvGeom_transparent = field _mjvGeom "transparent" mjtByte

  let () = seal _mjvGeom

  type mjvGeom = _mjvGeom

  let mjvGeom = _mjvGeom

  type _mjvLight

  let _mjvLight : _mjvLight structure typ = structure "_mjvLight"

  (**  position rel. to body frame *)
  let mjvLight_pos = field _mjvLight "pos" (ptr float)

  (**  direction rel. to body frame *)
  let mjvLight_dir = field _mjvLight "dir" (ptr float)

  (**  OpenGL attenuation (quadratic model) *)
  let mjvLight_attenuation = field _mjvLight "attenuation" (ptr float)

  (**  OpenGL cutoff *)
  let mjvLight_cutoff = field _mjvLight "cutoff" float

  (**  OpenGL exponent *)
  let mjvLight_exponent = field _mjvLight "exponent" float

  (**  ambient rgb (alpha=1) *)
  let mjvLight_ambient = field _mjvLight "ambient" (ptr float)

  (**  diffuse rgb (alpha=1) *)
  let mjvLight_diffuse = field _mjvLight "diffuse" (ptr float)

  (**  specular rgb (alpha=1) *)
  let mjvLight_specular = field _mjvLight "specular" (ptr float)

  (**  headlight *)
  let mjvLight_headlight = field _mjvLight "headlight" mjtByte

  (**  directional light *)
  let mjvLight_directional = field _mjvLight "directional" mjtByte

  (**  does light cast shadows *)
  let mjvLight_castshadow = field _mjvLight "castshadow" mjtByte

  let () = seal _mjvLight

  type mjvLight = _mjvLight

  let mjvLight = _mjvLight

  type _mjvOption

  let _mjvOption : _mjvOption structure typ = structure "_mjvOption"

  (**  what objects to label (mjtLabel) *)
  let mjvOption_label = field _mjvOption "label" int

  (**  which frame to show (mjtFrame) *)
  let mjvOption_frame = field _mjvOption "frame" int

  (**  geom visualization by group *)
  let mjvOption_geomgroup = field _mjvOption "geomgroup" (ptr mjtByte)

  (**  site visualization by group *)
  let mjvOption_sitegroup = field _mjvOption "sitegroup" (ptr mjtByte)

  (**  joint visualization by group *)
  let mjvOption_jointgroup = field _mjvOption "jointgroup" (ptr mjtByte)

  (**  tendon visualization by group *)
  let mjvOption_tendongroup = field _mjvOption "tendongroup" (ptr mjtByte)

  (**  actuator visualization by group *)
  let mjvOption_actuatorgroup = field _mjvOption "actuatorgroup" (ptr mjtByte)

  (**  visualization flags (indexed by mjtVisFlag) *)
  let mjvOption_flags = field _mjvOption "flags" (ptr mjtByte)

  let () = seal _mjvOption

  type mjvOption = _mjvOption

  let mjvOption = _mjvOption

  type _mjvScene

  let _mjvScene : _mjvScene structure typ = structure "_mjvScene"

  (**  size of allocated geom buffer *)
  let mjvScene_maxgeom = field _mjvScene "maxgeom" int

  (**  number of geoms currently in buffer *)
  let mjvScene_ngeom = field _mjvScene "ngeom" int

  (**  buffer for geoms *)
  let mjvScene_geoms = field _mjvScene "geoms" (ptr mjvGeom)

  (**  buffer for ordering geoms by distance to camera *)
  let mjvScene_geomorder = field _mjvScene "geomorder" (ptr int)

  (**  number of skins *)
  let mjvScene_nskin = field _mjvScene "nskin" int

  (**  number of faces in skin *)
  let mjvScene_skinfacenum = field _mjvScene "skinfacenum" (ptr int)

  (**  address of skin vertices *)
  let mjvScene_skinvertadr = field _mjvScene "skinvertadr" (ptr int)

  (**  number of vertices in skin *)
  let mjvScene_skinvertnum = field _mjvScene "skinvertnum" (ptr int)

  (**  skin vertex data *)
  let mjvScene_skinvert = field _mjvScene "skinvert" (ptr float)

  (**  skin normal data *)
  let mjvScene_skinnormal = field _mjvScene "skinnormal" (ptr float)

  (**  number of lights currently in buffer *)
  let mjvScene_nlight = field _mjvScene "nlight" int

  (**  buffer for lights *)
  let mjvScene_lights = field _mjvScene "lights" (ptr mjvLight)

  (**  left and right camera *)
  let mjvScene_camera = field _mjvScene "camera" (ptr mjvGLCamera)

  (**  enable model transformation *)
  let mjvScene_enabletransform = field _mjvScene "enabletransform" mjtByte

  (**  model translation *)
  let mjvScene_translate = field _mjvScene "translate" (ptr float)

  (**  model quaternion rotation *)
  let mjvScene_rotate = field _mjvScene "rotate" (ptr float)

  (**  model scaling *)
  let mjvScene_scale = field _mjvScene "scale" float

  (**  stereoscopic rendering (mjtStereo) *)
  let mjvScene_stereo = field _mjvScene "stereo" int

  (**  rendering flags (indexed by mjtRndFlag) *)
  let mjvScene_flags = field _mjvScene "flags" (ptr mjtByte)

  (**  frame pixel width; 0: disable framing *)
  let mjvScene_framewidth = field _mjvScene "framewidth" int

  (**  frame color *)
  let mjvScene_framergb = field _mjvScene "framergb" (ptr float)

  let () = seal _mjvScene

  type mjvScene = _mjvScene

  let mjvScene = _mjvScene

  type _mjvFigure

  let _mjvFigure : _mjvFigure structure typ = structure "_mjvFigure"

  (**  show legend *)
  let mjvFigure_flg_legend = field _mjvFigure "flg_legend" int

  (**  show grid tick labels (x,y) *)
  let mjvFigure_flg_ticklabel = field _mjvFigure "flg_ticklabel" (ptr int)

  (**  automatically extend axis ranges to fit data *)
  let mjvFigure_flg_extend = field _mjvFigure "flg_extend" int

  (**  isolated line segments (i.e. GL_LINES) *)
  let mjvFigure_flg_barplot = field _mjvFigure "flg_barplot" int

  (**  vertical selection line *)
  let mjvFigure_flg_selection = field _mjvFigure "flg_selection" int

  (**  symmetric y-axis *)
  let mjvFigure_flg_symmetric = field _mjvFigure "flg_symmetric" int

  (**  line width *)
  let mjvFigure_linewidth = field _mjvFigure "linewidth" float

  (**  grid line width *)
  let mjvFigure_gridwidth = field _mjvFigure "gridwidth" float

  (**  number of grid points in (x,y) *)
  let mjvFigure_gridsize = field _mjvFigure "gridsize" (ptr int)

  (**  grid line rgb *)
  let mjvFigure_gridrgb = field _mjvFigure "gridrgb" (ptr float)

  (**  figure color and alpha *)
  let mjvFigure_figurergba = field _mjvFigure "figurergba" (ptr float)

  (**  pane color and alpha *)
  let mjvFigure_panergba = field _mjvFigure "panergba" (ptr float)

  (**  legend color and alpha *)
  let mjvFigure_legendrgba = field _mjvFigure "legendrgba" (ptr float)

  (**  text color *)
  let mjvFigure_textrgb = field _mjvFigure "textrgb" (ptr float)

  (**  x-tick label format for sprintf *)
  let mjvFigure_xformat = field _mjvFigure "xformat" string

  (**  y-tick label format for sprintf *)
  let mjvFigure_yformat = field _mjvFigure "yformat" string

  (**  string used to determine min y-tick width *)
  let mjvFigure_minwidth = field _mjvFigure "minwidth" string

  (**  figure title; subplots separated with 2+ spaces *)
  let mjvFigure_title = field _mjvFigure "title" string

  (**  x-axis label *)
  let mjvFigure_xlabel = field _mjvFigure "xlabel" string

  (**  number of lines to offset legend *)
  let mjvFigure_legendoffset = field _mjvFigure "legendoffset" int

  (**  selected subplot (for title rendering) *)
  let mjvFigure_subplot = field _mjvFigure "subplot" int

  (**  if point is in legend rect, highlight line *)
  let mjvFigure_highlight = field _mjvFigure "highlight" (ptr int)

  (**  if id>=0 and no point, highlight id *)
  let mjvFigure_highlightid = field _mjvFigure "highlightid" int

  (**  selection line x-value *)
  let mjvFigure_selection = field _mjvFigure "selection" float

  (**  number of points in line; (0) disable *)
  let mjvFigure_linepnt = field _mjvFigure "linepnt" (ptr int)

  (**  range of x-axis in pixels *)
  let mjvFigure_xaxispixel = field _mjvFigure "xaxispixel" (ptr int)

  (**  range of y-axis in pixels *)
  let mjvFigure_yaxispixel = field _mjvFigure "yaxispixel" (ptr int)

  (**  range of x-axis in data units *)
  let mjvFigure_xaxisdata = field _mjvFigure "xaxisdata" (ptr float)

  (**  range of y-axis in data units *)
  let mjvFigure_yaxisdata = field _mjvFigure "yaxisdata" (ptr float)

  let () = seal _mjvFigure

  type mjvFigure = _mjvFigure

  let mjvFigure = _mjvFigure
  (* -------------------------------------------------------------------------------- *)
  (* ------------------------------ mjrender.h -------------------------------------- *)
  (* -------------------------------------------------------------------------------- *)

  (**  grid position for overlay *)
  type mjtGridPos =
    | MjGRID_TOPLEFT
    | MjGRID_TOPRIGHT
    | MjGRID_BOTTOMLEFT
    | MjGRID_BOTTOMRIGHT

  let mjGRID_TOPLEFT = constant "mjGRID_TOPLEFT" int64_t
  let mjGRID_TOPRIGHT = constant "mjGRID_TOPRIGHT" int64_t
  let mjGRID_BOTTOMLEFT = constant "mjGRID_BOTTOMLEFT" int64_t
  let mjGRID_BOTTOMRIGHT = constant "mjGRID_BOTTOMRIGHT" int64_t

  let mjtGridPos =
    S.enum
      "mjtGridPos"
      ~typedef:true
      [ MjGRID_TOPLEFT, mjGRID_TOPLEFT
      ; MjGRID_TOPRIGHT, mjGRID_TOPRIGHT
      ; MjGRID_BOTTOMLEFT, mjGRID_BOTTOMLEFT
      ; MjGRID_BOTTOMRIGHT, mjGRID_BOTTOMRIGHT
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtGridPos element data type enum")


  (**  OpenGL framebuffer option *)
  type mjtFramebuffer =
    | MjFB_WINDOW
    | MjFB_OFFSCREEN

  let mjFB_WINDOW = constant "mjFB_WINDOW" int64_t
  let mjFB_OFFSCREEN = constant "mjFB_OFFSCREEN" int64_t

  let mjtFramebuffer =
    S.enum
      "mjtFramebuffer"
      ~typedef:true
      [ MjFB_WINDOW, mjFB_WINDOW; MjFB_OFFSCREEN, mjFB_OFFSCREEN ]
      ~unexpected:(fun _ -> failwith "unexpected mjtFramebuffer element data type enum")


  (**  font scale, used at context creation *)
  type mjtFontScale =
    | MjFONTSCALE_50
    | MjFONTSCALE_100
    | MjFONTSCALE_150
    | MjFONTSCALE_200
    | MjFONTSCALE_250
    | MjFONTSCALE_300

  let mjFONTSCALE_50 = constant "mjFONTSCALE_50" int64_t
  let mjFONTSCALE_100 = constant "mjFONTSCALE_100" int64_t
  let mjFONTSCALE_150 = constant "mjFONTSCALE_150" int64_t
  let mjFONTSCALE_200 = constant "mjFONTSCALE_200" int64_t
  let mjFONTSCALE_250 = constant "mjFONTSCALE_250" int64_t
  let mjFONTSCALE_300 = constant "mjFONTSCALE_300" int64_t

  let mjtFontScale =
    S.enum
      "mjtFontScale"
      ~typedef:true
      [ MjFONTSCALE_50, mjFONTSCALE_50
      ; MjFONTSCALE_100, mjFONTSCALE_100
      ; MjFONTSCALE_150, mjFONTSCALE_150
      ; MjFONTSCALE_200, mjFONTSCALE_200
      ; MjFONTSCALE_250, mjFONTSCALE_250
      ; MjFONTSCALE_300, mjFONTSCALE_300
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtFontScale element data type enum")


  (**  font type, used at each text operation *)
  type mjtFont =
    | MjFONT_NORMAL
    | MjFONT_SHADOW
    | MjFONT_BIG

  let mjFONT_NORMAL = constant "mjFONT_NORMAL" int64_t
  let mjFONT_SHADOW = constant "mjFONT_SHADOW" int64_t
  let mjFONT_BIG = constant "mjFONT_BIG" int64_t

  let mjtFont =
    S.enum
      "mjtFont"
      ~typedef:true
      [ MjFONT_NORMAL, mjFONT_NORMAL
      ; MjFONT_SHADOW, mjFONT_SHADOW
      ; MjFONT_BIG, mjFONT_BIG
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtFont element data type enum")


  type _mjrRect

  let _mjrRect : _mjrRect structure typ = structure "_mjrRect"

  (**  left (usually 0) *)
  let mjrRect_left = field _mjrRect "left" int

  (**  bottom (usually 0) *)
  let mjrRect_bottom = field _mjrRect "bottom" int

  (**  width (usually buffer width) *)
  let mjrRect_width = field _mjrRect "width" int

  (**  height (usually buffer height) *)
  let mjrRect_height = field _mjrRect "height" int

  let () = seal _mjrRect

  type mjrRect = _mjrRect

  let mjrRect = _mjrRect

  type _mjrContext

  let _mjrContext : _mjrContext structure typ = structure "_mjrContext"

  (**  line width for wireframe rendering *)
  let mjrContext_lineWidth = field _mjrContext "lineWidth" float

  (**  clipping radius for directional lights *)
  let mjrContext_shadowClip = field _mjrContext "shadowClip" float

  (**  fraction of light cutoff for spot lights *)
  let mjrContext_shadowScale = field _mjrContext "shadowScale" float

  (**  fog start = stat.extent * vis.map.fogstart *)
  let mjrContext_fogStart = field _mjrContext "fogStart" float

  (**  fog end = stat.extent * vis.map.fogend *)
  let mjrContext_fogEnd = field _mjrContext "fogEnd" float

  (**  fog rgba *)
  let mjrContext_fogRGBA = field _mjrContext "fogRGBA" (ptr float)

  (**  size of shadow map texture *)
  let mjrContext_shadowSize = field _mjrContext "shadowSize" int

  (**  width of offscreen buffer *)
  let mjrContext_offWidth = field _mjrContext "offWidth" int

  (**  height of offscreen buffer *)
  let mjrContext_offHeight = field _mjrContext "offHeight" int

  (**  number of offscreen buffer multisamples *)
  let mjrContext_offSamples = field _mjrContext "offSamples" int

  (**  font scale *)
  let mjrContext_fontScale = field _mjrContext "fontScale" int

  (**  auxiliary buffer width *)
  let mjrContext_auxWidth = field _mjrContext "auxWidth" (ptr int)

  (**  auxiliary buffer height *)
  let mjrContext_auxHeight = field _mjrContext "auxHeight" (ptr int)

  (**  auxiliary buffer multisamples *)
  let mjrContext_auxSamples = field _mjrContext "auxSamples" (ptr int)

  (**  number of allocated textures *)
  let mjrContext_ntexture = field _mjrContext "ntexture" int

  (**  type of texture (mjtTexture) *)
  let mjrContext_textureType = field _mjrContext "textureType" (ptr int)

  (**  all planes from model *)
  let mjrContext_rangePlane = field _mjrContext "rangePlane" int

  (**  all meshes from model *)
  let mjrContext_rangeMesh = field _mjrContext "rangeMesh" int

  (**  all hfields from model *)
  let mjrContext_rangeHField = field _mjrContext "rangeHField" int

  (**  all builtin geoms, with quality from model *)
  let mjrContext_rangeBuiltin = field _mjrContext "rangeBuiltin" int

  (**  all characters in font *)
  let mjrContext_rangeFont = field _mjrContext "rangeFont" int

  (**  number of skins *)
  let mjrContext_nskin = field _mjrContext "nskin" int

  (**  character widths: normal and shadow *)
  let mjrContext_charWidth = field _mjrContext "charWidth" (ptr int)

  (**  chacarter widths: big *)
  let mjrContext_charWidthBig = field _mjrContext "charWidthBig" (ptr int)

  (**  character heights: normal and shadow *)
  let mjrContext_charHeight = field _mjrContext "charHeight" int

  (**  character heights: big *)
  let mjrContext_charHeightBig = field _mjrContext "charHeightBig" int

  (**  is glew initialized *)
  let mjrContext_glewInitialized = field _mjrContext "glewInitialized" int

  (**  is default/window framebuffer available *)
  let mjrContext_windowAvailable = field _mjrContext "windowAvailable" int

  (**  number of samples for default/window framebuffer *)
  let mjrContext_windowSamples = field _mjrContext "windowSamples" int

  (**  is stereo available for default/window framebuffer *)
  let mjrContext_windowStereo = field _mjrContext "windowStereo" int

  (**  is default/window framebuffer double buffered *)
  let mjrContext_windowDoublebuffer = field _mjrContext "windowDoublebuffer" int

  (**  currently active framebuffer: mjFB_WINDOW or mjFB_OFFSCREEN *)
  let mjrContext_currentBuffer = field _mjrContext "currentBuffer" int

  let () = seal _mjrContext

  type mjrContext = _mjrContext

  let mjrContext = _mjrContext
  (* -------------------------------------------------------------------------------- *)
  (* ------------------------------ mjui.h ------------------------------------------ *)
  (* -------------------------------------------------------------------------------- *)

  (**  mouse button *)
  type mjtButton =
    | MjBUTTON_NONE
    | MjBUTTON_LEFT
    | MjBUTTON_RIGHT
    | MjBUTTON_MIDDLE

  let mjBUTTON_NONE = constant "mjBUTTON_NONE" int64_t
  let mjBUTTON_LEFT = constant "mjBUTTON_LEFT" int64_t
  let mjBUTTON_RIGHT = constant "mjBUTTON_RIGHT" int64_t
  let mjBUTTON_MIDDLE = constant "mjBUTTON_MIDDLE" int64_t

  let mjtButton =
    S.enum
      "mjtButton"
      ~typedef:true
      [ MjBUTTON_NONE, mjBUTTON_NONE
      ; MjBUTTON_LEFT, mjBUTTON_LEFT
      ; MjBUTTON_RIGHT, mjBUTTON_RIGHT
      ; MjBUTTON_MIDDLE, mjBUTTON_MIDDLE
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtButton element data type enum")


  (**  mouse and keyboard event type *)
  type mjtEvent =
    | MjEVENT_NONE
    | MjEVENT_MOVE
    | MjEVENT_PRESS
    | MjEVENT_RELEASE
    | MjEVENT_SCROLL
    | MjEVENT_KEY
    | MjEVENT_RESIZE

  let mjEVENT_NONE = constant "mjEVENT_NONE" int64_t
  let mjEVENT_MOVE = constant "mjEVENT_MOVE" int64_t
  let mjEVENT_PRESS = constant "mjEVENT_PRESS" int64_t
  let mjEVENT_RELEASE = constant "mjEVENT_RELEASE" int64_t
  let mjEVENT_SCROLL = constant "mjEVENT_SCROLL" int64_t
  let mjEVENT_KEY = constant "mjEVENT_KEY" int64_t
  let mjEVENT_RESIZE = constant "mjEVENT_RESIZE" int64_t

  let mjtEvent =
    S.enum
      "mjtEvent"
      ~typedef:true
      [ MjEVENT_NONE, mjEVENT_NONE
      ; MjEVENT_MOVE, mjEVENT_MOVE
      ; MjEVENT_PRESS, mjEVENT_PRESS
      ; MjEVENT_RELEASE, mjEVENT_RELEASE
      ; MjEVENT_SCROLL, mjEVENT_SCROLL
      ; MjEVENT_KEY, mjEVENT_KEY
      ; MjEVENT_RESIZE, mjEVENT_RESIZE
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtEvent element data type enum")


  (**  UI item type *)
  type mjtItem =
    | MjITEM_END
    | MjITEM_SECTION
    | MjITEM_SEPARATOR
    | MjITEM_STATIC
    | MjITEM_BUTTON
    | MjITEM_CHECKINT
    | MjITEM_CHECKBYTE
    | MjITEM_RADIO
    | MjITEM_RADIOLINE
    | MjITEM_SELECT
    | MjITEM_SLIDERINT
    | MjITEM_SLIDERNUM
    | MjITEM_EDITINT
    | MjITEM_EDITNUM
    | MjITEM_EDITTXT
    | MjNITEM

  let mjITEM_END = constant "mjITEM_END" int64_t
  let mjITEM_SECTION = constant "mjITEM_SECTION" int64_t
  let mjITEM_SEPARATOR = constant "mjITEM_SEPARATOR" int64_t
  let mjITEM_STATIC = constant "mjITEM_STATIC" int64_t
  let mjITEM_BUTTON = constant "mjITEM_BUTTON" int64_t
  let mjITEM_CHECKINT = constant "mjITEM_CHECKINT" int64_t
  let mjITEM_CHECKBYTE = constant "mjITEM_CHECKBYTE" int64_t
  let mjITEM_RADIO = constant "mjITEM_RADIO" int64_t
  let mjITEM_RADIOLINE = constant "mjITEM_RADIOLINE" int64_t
  let mjITEM_SELECT = constant "mjITEM_SELECT" int64_t
  let mjITEM_SLIDERINT = constant "mjITEM_SLIDERINT" int64_t
  let mjITEM_SLIDERNUM = constant "mjITEM_SLIDERNUM" int64_t
  let mjITEM_EDITINT = constant "mjITEM_EDITINT" int64_t
  let mjITEM_EDITNUM = constant "mjITEM_EDITNUM" int64_t
  let mjITEM_EDITTXT = constant "mjITEM_EDITTXT" int64_t
  let mjNITEM = constant "mjNITEM" int64_t

  let mjtItem =
    S.enum
      "mjtItem"
      ~typedef:true
      [ MjITEM_END, mjITEM_END
      ; MjITEM_SECTION, mjITEM_SECTION
      ; MjITEM_SEPARATOR, mjITEM_SEPARATOR
      ; MjITEM_STATIC, mjITEM_STATIC
      ; MjITEM_BUTTON, mjITEM_BUTTON
      ; MjITEM_CHECKINT, mjITEM_CHECKINT
      ; MjITEM_CHECKBYTE, mjITEM_CHECKBYTE
      ; MjITEM_RADIO, mjITEM_RADIO
      ; MjITEM_RADIOLINE, mjITEM_RADIOLINE
      ; MjITEM_SELECT, mjITEM_SELECT
      ; MjITEM_SLIDERINT, mjITEM_SLIDERINT
      ; MjITEM_SLIDERNUM, mjITEM_SLIDERNUM
      ; MjITEM_EDITINT, mjITEM_EDITINT
      ; MjITEM_EDITNUM, mjITEM_EDITNUM
      ; MjITEM_EDITTXT, mjITEM_EDITTXT
      ; MjNITEM, mjNITEM
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtItem element data type enum")


  type _mjuiState

  let _mjuiState : _mjuiState structure typ = structure "_mjuiState"

  (**  number of rectangles used *)
  let mjuiState_nrect = field _mjuiState "nrect" int

  (**  rectangles (index 0: entire window) *)
  let mjuiState_rect = field _mjuiState "rect" (ptr mjrRect)

  (**  pointer to user data (for callbacks) *)
  let mjuiState_userdata = field _mjuiState "userdata" (ptr void)

  (**  (type mjtEvent) *)
  let mjuiState_type = field _mjuiState "type" int

  (**  is left button down *)
  let mjuiState_left = field _mjuiState "left" int

  (**  is right button down *)
  let mjuiState_right = field _mjuiState "right" int

  (**  is middle button down *)
  let mjuiState_middle = field _mjuiState "middle" int

  (**  is last press a double click *)
  let mjuiState_doubleclick = field _mjuiState "doubleclick" int

  (**  which button was pressed (mjtButton) *)
  let mjuiState_button = field _mjuiState "button" int

  (**  time of last button press *)
  let mjuiState_buttontime = field _mjuiState "buttontime" double

  (**  x position *)
  let mjuiState_x = field _mjuiState "x" double

  (**  y position *)
  let mjuiState_y = field _mjuiState "y" double

  (**  x displacement *)
  let mjuiState_dx = field _mjuiState "dx" double

  (**  y displacement *)
  let mjuiState_dy = field _mjuiState "dy" double

  (**  x scroll *)
  let mjuiState_sx = field _mjuiState "sx" double

  (**  y scroll *)
  let mjuiState_sy = field _mjuiState "sy" double

  (**  is control down *)
  let mjuiState_control = field _mjuiState "control" int

  (**  is shift down *)
  let mjuiState_shift = field _mjuiState "shift" int

  (**  is alt down *)
  let mjuiState_alt = field _mjuiState "alt" int

  (**  which key was pressed *)
  let mjuiState_key = field _mjuiState "key" int

  (**  time of last key press *)
  let mjuiState_keytime = field _mjuiState "keytime" double

  (**  which rectangle contains mouse *)
  let mjuiState_mouserect = field _mjuiState "mouserect" int

  (**  which rectangle is dragged with mouse *)
  let mjuiState_dragrect = field _mjuiState "dragrect" int

  (**  which button started drag (mjtButton) *)
  let mjuiState_dragbutton = field _mjuiState "dragbutton" int

  let () = seal _mjuiState

  type mjuiState = _mjuiState

  let mjuiState = _mjuiState

  type _mjuiThemeSpacing

  let _mjuiThemeSpacing : _mjuiThemeSpacing structure typ = structure "_mjuiThemeSpacing"

  (**  total width *)
  let mjuiThemeSpacing_total = field _mjuiThemeSpacing "total" int

  (**  scrollbar width *)
  let mjuiThemeSpacing_scroll = field _mjuiThemeSpacing "scroll" int

  (**  label width *)
  let mjuiThemeSpacing_label = field _mjuiThemeSpacing "label" int

  (**  section gap *)
  let mjuiThemeSpacing_section = field _mjuiThemeSpacing "section" int

  (**  item side gap *)
  let mjuiThemeSpacing_itemside = field _mjuiThemeSpacing "itemside" int

  (**  item middle gap *)
  let mjuiThemeSpacing_itemmid = field _mjuiThemeSpacing "itemmid" int

  (**  item vertical gap *)
  let mjuiThemeSpacing_itemver = field _mjuiThemeSpacing "itemver" int

  (**  text horizontal gap *)
  let mjuiThemeSpacing_texthor = field _mjuiThemeSpacing "texthor" int

  (**  text vertical gap *)
  let mjuiThemeSpacing_textver = field _mjuiThemeSpacing "textver" int

  (**  number of pixels to scroll *)
  let mjuiThemeSpacing_linescroll = field _mjuiThemeSpacing "linescroll" int

  (**  number of multisamples *)
  let mjuiThemeSpacing_samples = field _mjuiThemeSpacing "samples" int

  let () = seal _mjuiThemeSpacing

  type mjuiThemeSpacing = _mjuiThemeSpacing

  let mjuiThemeSpacing = _mjuiThemeSpacing

  type _mjuiThemeColor

  let _mjuiThemeColor : _mjuiThemeColor structure typ = structure "_mjuiThemeColor"

  (**  master background *)
  let mjuiThemeColor_master = field _mjuiThemeColor "master" (ptr float)

  (**  scrollbar thumb *)
  let mjuiThemeColor_thumb = field _mjuiThemeColor "thumb" (ptr float)

  (**  section title *)
  let mjuiThemeColor_secttitle = field _mjuiThemeColor "secttitle" (ptr float)

  (**  section font *)
  let mjuiThemeColor_sectfont = field _mjuiThemeColor "sectfont" (ptr float)

  (**  section symbol *)
  let mjuiThemeColor_sectsymbol = field _mjuiThemeColor "sectsymbol" (ptr float)

  (**  section pane *)
  let mjuiThemeColor_sectpane = field _mjuiThemeColor "sectpane" (ptr float)

  (**  shortcut background *)
  let mjuiThemeColor_shortcut = field _mjuiThemeColor "shortcut" (ptr float)

  (**  font active *)
  let mjuiThemeColor_fontactive = field _mjuiThemeColor "fontactive" (ptr float)

  (**  font inactive *)
  let mjuiThemeColor_fontinactive = field _mjuiThemeColor "fontinactive" (ptr float)

  (**  decor inactive *)
  let mjuiThemeColor_decorinactive = field _mjuiThemeColor "decorinactive" (ptr float)

  (**  inactive slider color 2 *)
  let mjuiThemeColor_decorinactive2 = field _mjuiThemeColor "decorinactive2" (ptr float)

  (**  button *)
  let mjuiThemeColor_button = field _mjuiThemeColor "button" (ptr float)

  (**  check *)
  let mjuiThemeColor_check = field _mjuiThemeColor "check" (ptr float)

  (**  radio *)
  let mjuiThemeColor_radio = field _mjuiThemeColor "radio" (ptr float)

  (**  select *)
  let mjuiThemeColor_select = field _mjuiThemeColor "select" (ptr float)

  (**  select pane *)
  let mjuiThemeColor_select2 = field _mjuiThemeColor "select2" (ptr float)

  (**  slider *)
  let mjuiThemeColor_slider = field _mjuiThemeColor "slider" (ptr float)

  (**  slider color 2 *)
  let mjuiThemeColor_slider2 = field _mjuiThemeColor "slider2" (ptr float)

  (**  edit *)
  let mjuiThemeColor_edit = field _mjuiThemeColor "edit" (ptr float)

  (**  edit invalid *)
  let mjuiThemeColor_edit2 = field _mjuiThemeColor "edit2" (ptr float)

  (**  edit cursor *)
  let mjuiThemeColor_cursor = field _mjuiThemeColor "cursor" (ptr float)

  let () = seal _mjuiThemeColor

  type mjuiThemeColor = _mjuiThemeColor

  let mjuiThemeColor = _mjuiThemeColor

  type _mjuiItemSingle

  let _mjuiItemSingle : _mjuiItemSingle structure typ = structure "_mjuiItemSingle"

  (**  0: none, 1: control, 2: shift; 4: alt *)
  let mjuiItemSingle_modifier = field _mjuiItemSingle "modifier" int

  (**  shortcut key; 0: undefined *)
  let mjuiItemSingle_shortcut = field _mjuiItemSingle "shortcut" int

  let () = seal _mjuiItemSingle

  type mjuiItemSingle = _mjuiItemSingle

  let mjuiItemSingle = _mjuiItemSingle

  type _mjuiItemMulti

  let _mjuiItemMulti : _mjuiItemMulti structure typ = structure "_mjuiItemMulti"

  (**  number of elements in group *)
  let mjuiItemMulti_nelem = field _mjuiItemMulti "nelem" int

  let () = seal _mjuiItemMulti

  type mjuiItemMulti = _mjuiItemMulti

  let mjuiItemMulti = _mjuiItemMulti

  type _mjuiItemSlider

  let _mjuiItemSlider : _mjuiItemSlider structure typ = structure "_mjuiItemSlider"

  (**  slider range *)
  let mjuiItemSlider_range = field _mjuiItemSlider "range" (ptr double)

  (**  number of range divisions *)
  let mjuiItemSlider_divisions = field _mjuiItemSlider "divisions" double

  let () = seal _mjuiItemSlider

  type mjuiItemSlider = _mjuiItemSlider

  let mjuiItemSlider = _mjuiItemSlider

  type _mjuiItemEdit

  let _mjuiItemEdit : _mjuiItemEdit structure typ = structure "_mjuiItemEdit"

  (**  number of elements in list *)
  let mjuiItemEdit_nelem = field _mjuiItemEdit "nelem" int

  let () = seal _mjuiItemEdit

  type mjuiItemEdit = _mjuiItemEdit

  let mjuiItemEdit = _mjuiItemEdit

  type _mjuiItem

  let _mjuiItem : _mjuiItem structure typ = structure "_mjuiItem"

  (**  type (mjtItem) *)
  let mjuiItem_type = field _mjuiItem "type" int

  (**  name *)
  let mjuiItem_name = field _mjuiItem "name" string

  (**  0: disable, 1: enable, 2+: use predicate *)
  let mjuiItem_state = field _mjuiItem "state" int

  (**  id of section containing item *)
  let mjuiItem_sectionid = field _mjuiItem "sectionid" int

  (**  id of item within section *)
  let mjuiItem_itemid = field _mjuiItem "itemid" int

  (**  rectangle occupied by item *)
  let mjuiItem_rect = field _mjuiItem "rect" mjrRect

  let () = seal _mjuiItem

  type mjuiItem = _mjuiItem

  let mjuiItem = _mjuiItem

  type _mjuiSection

  let _mjuiSection : _mjuiSection structure typ = structure "_mjuiSection"

  (**  name *)
  let mjuiSection_name = field _mjuiSection "name" string

  (**  0: closed, 1: open *)
  let mjuiSection_state = field _mjuiSection "state" int

  (**  0: none, 1: control, 2: shift; 4: alt *)
  let mjuiSection_modifier = field _mjuiSection "modifier" int

  (**  shortcut key; 0: undefined *)
  let mjuiSection_shortcut = field _mjuiSection "shortcut" int

  (**  number of items in use *)
  let mjuiSection_nitem = field _mjuiSection "nitem" int

  (**  preallocated array of items *)
  let mjuiSection_item = field _mjuiSection "item" (ptr mjuiItem)

  (**  rectangle occupied by title *)
  let mjuiSection_rtitle = field _mjuiSection "rtitle" mjrRect

  (**  rectangle occupied by content *)
  let mjuiSection_rcontent = field _mjuiSection "rcontent" mjrRect

  let () = seal _mjuiSection

  type mjuiSection = _mjuiSection

  let mjuiSection = _mjuiSection

  type _mjUI

  let _mjUI : _mjUI structure typ = structure "_mjUI"

  (**  UI theme spacing *)
  let mjUI_spacing = field _mjUI "spacing" mjuiThemeSpacing

  (**  UI theme color *)
  let mjUI_color = field _mjUI "color" mjuiThemeColor

  (**  callback to set item state programmatically *)
  let mjUI_predicate = field _mjUI "predicate" mjfItemEnable

  (**  pointer to user data (passed to predicate) *)
  let mjUI_userdata = field _mjUI "userdata" (ptr void)

  (**  index of this ui rectangle in mjuiState *)
  let mjUI_rectid = field _mjUI "rectid" int

  (**  aux buffer index of this ui *)
  let mjUI_auxid = field _mjUI "auxid" int

  (**  number of radio columns (0 defaults to 2) *)
  let mjUI_radiocol = field _mjUI "radiocol" int

  (**  width *)
  let mjUI_width = field _mjUI "width" int

  (**  current heigth *)
  let mjUI_height = field _mjUI "height" int

  (**  height when all sections open *)
  let mjUI_maxheight = field _mjUI "maxheight" int

  (**  scroll from top of UI *)
  let mjUI_scroll = field _mjUI "scroll" int

  (**  0: none, -1: scroll, otherwise 1+section *)
  let mjUI_mousesect = field _mjUI "mousesect" int

  (**  item within section *)
  let mjUI_mouseitem = field _mjUI "mouseitem" int

  (**  help button down: print shortcuts *)
  let mjUI_mousehelp = field _mjUI "mousehelp" int

  (**  0: none, otherwise 1+section *)
  let mjUI_editsect = field _mjUI "editsect" int

  (**  item within section *)
  let mjUI_edititem = field _mjUI "edititem" int

  (**  cursor position *)
  let mjUI_editcursor = field _mjUI "editcursor" int

  (**  horizontal scroll *)
  let mjUI_editscroll = field _mjUI "editscroll" int

  (**  current text *)
  let mjUI_edittext = field _mjUI "edittext" string

  (**  pointer to changed edit in last mjui_event *)
  let mjUI_editchanged = field _mjUI "editchanged" (ptr mjuiItem)

  (**  number of sections in use *)
  let mjUI_nsect = field _mjUI "nsect" int

  (**  preallocated array of sections *)
  let mjUI_sect = field _mjUI "sect" (ptr mjuiSection)

  let () = seal _mjUI

  type mjUI = _mjUI

  let mjUI = _mjUI

  type _mjuiDef

  let _mjuiDef : _mjuiDef structure typ = structure "_mjuiDef"

  (**  type (mjtItem); -1: section *)
  let mjuiDef_type = field _mjuiDef "type" int

  (**  name *)
  let mjuiDef_name = field _mjuiDef "name" string

  (**  state *)
  let mjuiDef_state = field _mjuiDef "state" int

  (**  pointer to data *)
  let mjuiDef_pdata = field _mjuiDef "pdata" (ptr void)

  (**  string with type-specific properties *)
  let mjuiDef_other = field _mjuiDef "other" string

  let () = seal _mjuiDef

  type mjuiDef = _mjuiDef

  let mjuiDef = _mjuiDef
end
