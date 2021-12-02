(* THIS FILE IS GENERATED AUTOMATICALLY, DO NOT EDIT BY HAND *)

open Ctypes

module Bindings (S : Cstubs.Types.TYPE) = struct
  open S

  type mjtByte = Unsigned.UChar.t

  let mjtByte = uchar

  type mjtNum = float

  let mjtNum = double
  let mjfItemEnable = static_funptr (int @-> ptr void @-> returning int)

  (* ------------------------- mjmodel.h ------------------------- *)

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
  let mjLROpt_mode = field _mjLROpt "mode" int
  let mjLROpt_useexisting = field _mjLROpt "useexisting" int
  let mjLROpt_uselimit = field _mjLROpt "uselimit" int
  let mjLROpt_accel = field _mjLROpt "accel" mjtNum
  let mjLROpt_maxforce = field _mjLROpt "maxforce" mjtNum
  let mjLROpt_timeconst = field _mjLROpt "timeconst" mjtNum
  let mjLROpt_timestep = field _mjLROpt "timestep" mjtNum
  let mjLROpt_inttotal = field _mjLROpt "inttotal" mjtNum
  let mjLROpt_inteval = field _mjLROpt "inteval" mjtNum
  let mjLROpt_tolrange = field _mjLROpt "tolrange" mjtNum
  let () = seal _mjLROpt

  type mjLROpt = _mjLROpt

  let mjLROpt = _mjLROpt

  type _mjVFS

  let _mjVFS : _mjVFS structure typ = structure "_mjVFS"
  let mjVFS_nfile = field _mjVFS "nfile" int
  let () = seal _mjVFS

  type mjVFS = _mjVFS

  let mjVFS = _mjVFS

  type _mjOption

  let _mjOption : _mjOption structure typ = structure "_mjOption"
  let mjOption_timestep = field _mjOption "timestep" mjtNum
  let mjOption_apirate = field _mjOption "apirate" mjtNum
  let mjOption_impratio = field _mjOption "impratio" mjtNum
  let mjOption_tolerance = field _mjOption "tolerance" mjtNum
  let mjOption_noslip_tolerance = field _mjOption "noslip_tolerance" mjtNum
  let mjOption_mpr_tolerance = field _mjOption "mpr_tolerance" mjtNum
  let mjOption_density = field _mjOption "density" mjtNum
  let mjOption_viscosity = field _mjOption "viscosity" mjtNum
  let mjOption_o_margin = field _mjOption "o_margin" mjtNum
  let mjOption_integrator = field _mjOption "integrator" int
  let mjOption_collision = field _mjOption "collision" int
  let mjOption_cone = field _mjOption "cone" int
  let mjOption_jacobian = field _mjOption "jacobian" int
  let mjOption_solver = field _mjOption "solver" int
  let mjOption_iterations = field _mjOption "iterations" int
  let mjOption_noslip_iterations = field _mjOption "noslip_iterations" int
  let mjOption_mpr_iterations = field _mjOption "mpr_iterations" int
  let mjOption_disableflags = field _mjOption "disableflags" int
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
  let mjStatistic_meaninertia = field _mjStatistic "meaninertia" mjtNum
  let mjStatistic_meanmass = field _mjStatistic "meanmass" mjtNum
  let mjStatistic_meansize = field _mjStatistic "meansize" mjtNum
  let mjStatistic_extent = field _mjStatistic "extent" mjtNum
  let () = seal _mjStatistic

  type mjStatistic = _mjStatistic

  let mjStatistic = _mjStatistic

  type _mjModel

  let _mjModel : _mjModel structure typ = structure "_mjModel"
  let mjModel_nq = field _mjModel "nq" int
  let mjModel_nv = field _mjModel "nv" int
  let mjModel_nu = field _mjModel "nu" int
  let mjModel_na = field _mjModel "na" int
  let mjModel_nbody = field _mjModel "nbody" int
  let mjModel_njnt = field _mjModel "njnt" int
  let mjModel_ngeom = field _mjModel "ngeom" int
  let mjModel_nsite = field _mjModel "nsite" int
  let mjModel_ncam = field _mjModel "ncam" int
  let mjModel_nlight = field _mjModel "nlight" int
  let mjModel_nmesh = field _mjModel "nmesh" int
  let mjModel_nmeshvert = field _mjModel "nmeshvert" int
  let mjModel_nmeshtexvert = field _mjModel "nmeshtexvert" int
  let mjModel_nmeshface = field _mjModel "nmeshface" int
  let mjModel_nmeshgraph = field _mjModel "nmeshgraph" int
  let mjModel_nskin = field _mjModel "nskin" int
  let mjModel_nskinvert = field _mjModel "nskinvert" int
  let mjModel_nskintexvert = field _mjModel "nskintexvert" int
  let mjModel_nskinface = field _mjModel "nskinface" int
  let mjModel_nskinbone = field _mjModel "nskinbone" int
  let mjModel_nskinbonevert = field _mjModel "nskinbonevert" int
  let mjModel_nhfield = field _mjModel "nhfield" int
  let mjModel_nhfielddata = field _mjModel "nhfielddata" int
  let mjModel_ntex = field _mjModel "ntex" int
  let mjModel_ntexdata = field _mjModel "ntexdata" int
  let mjModel_nmat = field _mjModel "nmat" int
  let mjModel_npair = field _mjModel "npair" int
  let mjModel_nexclude = field _mjModel "nexclude" int
  let mjModel_neq = field _mjModel "neq" int
  let mjModel_ntendon = field _mjModel "ntendon" int
  let mjModel_nwrap = field _mjModel "nwrap" int
  let mjModel_nsensor = field _mjModel "nsensor" int
  let mjModel_nnumeric = field _mjModel "nnumeric" int
  let mjModel_nnumericdata = field _mjModel "nnumericdata" int
  let mjModel_ntext = field _mjModel "ntext" int
  let mjModel_ntextdata = field _mjModel "ntextdata" int
  let mjModel_ntuple = field _mjModel "ntuple" int
  let mjModel_ntupledata = field _mjModel "ntupledata" int
  let mjModel_nkey = field _mjModel "nkey" int
  let mjModel_nmocap = field _mjModel "nmocap" int
  let mjModel_nuser_body = field _mjModel "nuser_body" int
  let mjModel_nuser_jnt = field _mjModel "nuser_jnt" int
  let mjModel_nuser_geom = field _mjModel "nuser_geom" int
  let mjModel_nuser_site = field _mjModel "nuser_site" int
  let mjModel_nuser_cam = field _mjModel "nuser_cam" int
  let mjModel_nuser_tendon = field _mjModel "nuser_tendon" int
  let mjModel_nuser_actuator = field _mjModel "nuser_actuator" int
  let mjModel_nuser_sensor = field _mjModel "nuser_sensor" int
  let mjModel_nnames = field _mjModel "nnames" int
  let mjModel_nM = field _mjModel "nM" int
  let mjModel_nemax = field _mjModel "nemax" int
  let mjModel_njmax = field _mjModel "njmax" int
  let mjModel_nconmax = field _mjModel "nconmax" int
  let mjModel_nstack = field _mjModel "nstack" int
  let mjModel_nuserdata = field _mjModel "nuserdata" int
  let mjModel_nsensordata = field _mjModel "nsensordata" int
  let mjModel_nbuffer = field _mjModel "nbuffer" int
  let mjModel_opt = field _mjModel "opt" mjOption
  let mjModel_vis = field _mjModel "vis" mjVisual
  let mjModel_stat = field _mjModel "stat" mjStatistic
  let mjModel_buffer = field _mjModel "buffer" (ptr void)
  let mjModel_qpos0 = field _mjModel "qpos0" (ptr mjtNum)
  let mjModel_qpos_spring = field _mjModel "qpos_spring" (ptr mjtNum)
  let mjModel_body_parentid = field _mjModel "body_parentid" (ptr int)
  let mjModel_body_rootid = field _mjModel "body_rootid" (ptr int)
  let mjModel_body_weldid = field _mjModel "body_weldid" (ptr int)
  let mjModel_body_mocapid = field _mjModel "body_mocapid" (ptr int)
  let mjModel_body_jntnum = field _mjModel "body_jntnum" (ptr int)
  let mjModel_body_jntadr = field _mjModel "body_jntadr" (ptr int)
  let mjModel_body_dofnum = field _mjModel "body_dofnum" (ptr int)
  let mjModel_body_dofadr = field _mjModel "body_dofadr" (ptr int)
  let mjModel_body_geomnum = field _mjModel "body_geomnum" (ptr int)
  let mjModel_body_geomadr = field _mjModel "body_geomadr" (ptr int)
  let mjModel_body_simple = field _mjModel "body_simple" (ptr mjtByte)
  let mjModel_body_sameframe = field _mjModel "body_sameframe" (ptr mjtByte)
  let mjModel_body_pos = field _mjModel "body_pos" (ptr mjtNum)
  let mjModel_body_quat = field _mjModel "body_quat" (ptr mjtNum)
  let mjModel_body_ipos = field _mjModel "body_ipos" (ptr mjtNum)
  let mjModel_body_iquat = field _mjModel "body_iquat" (ptr mjtNum)
  let mjModel_body_mass = field _mjModel "body_mass" (ptr mjtNum)
  let mjModel_body_subtreemass = field _mjModel "body_subtreemass" (ptr mjtNum)
  let mjModel_body_inertia = field _mjModel "body_inertia" (ptr mjtNum)
  let mjModel_body_invweight0 = field _mjModel "body_invweight0" (ptr mjtNum)
  let mjModel_body_user = field _mjModel "body_user" (ptr mjtNum)
  let mjModel_jnt_type = field _mjModel "jnt_type" (ptr int)
  let mjModel_jnt_qposadr = field _mjModel "jnt_qposadr" (ptr int)
  let mjModel_jnt_dofadr = field _mjModel "jnt_dofadr" (ptr int)
  let mjModel_jnt_bodyid = field _mjModel "jnt_bodyid" (ptr int)
  let mjModel_jnt_group = field _mjModel "jnt_group" (ptr int)
  let mjModel_jnt_limited = field _mjModel "jnt_limited" (ptr mjtByte)
  let mjModel_jnt_solref = field _mjModel "jnt_solref" (ptr mjtNum)
  let mjModel_jnt_solimp = field _mjModel "jnt_solimp" (ptr mjtNum)
  let mjModel_jnt_pos = field _mjModel "jnt_pos" (ptr mjtNum)
  let mjModel_jnt_axis = field _mjModel "jnt_axis" (ptr mjtNum)
  let mjModel_jnt_stiffness = field _mjModel "jnt_stiffness" (ptr mjtNum)
  let mjModel_jnt_range = field _mjModel "jnt_range" (ptr mjtNum)
  let mjModel_jnt_margin = field _mjModel "jnt_margin" (ptr mjtNum)
  let mjModel_jnt_user = field _mjModel "jnt_user" (ptr mjtNum)
  let mjModel_dof_bodyid = field _mjModel "dof_bodyid" (ptr int)
  let mjModel_dof_jntid = field _mjModel "dof_jntid" (ptr int)
  let mjModel_dof_parentid = field _mjModel "dof_parentid" (ptr int)
  let mjModel_dof_Madr = field _mjModel "dof_Madr" (ptr int)
  let mjModel_dof_simplenum = field _mjModel "dof_simplenum" (ptr int)
  let mjModel_dof_solref = field _mjModel "dof_solref" (ptr mjtNum)
  let mjModel_dof_solimp = field _mjModel "dof_solimp" (ptr mjtNum)
  let mjModel_dof_frictionloss = field _mjModel "dof_frictionloss" (ptr mjtNum)
  let mjModel_dof_armature = field _mjModel "dof_armature" (ptr mjtNum)
  let mjModel_dof_damping = field _mjModel "dof_damping" (ptr mjtNum)
  let mjModel_dof_invweight0 = field _mjModel "dof_invweight0" (ptr mjtNum)
  let mjModel_dof_M0 = field _mjModel "dof_M0" (ptr mjtNum)
  let mjModel_geom_type = field _mjModel "geom_type" (ptr int)
  let mjModel_geom_contype = field _mjModel "geom_contype" (ptr int)
  let mjModel_geom_conaffinity = field _mjModel "geom_conaffinity" (ptr int)
  let mjModel_geom_condim = field _mjModel "geom_condim" (ptr int)
  let mjModel_geom_bodyid = field _mjModel "geom_bodyid" (ptr int)
  let mjModel_geom_dataid = field _mjModel "geom_dataid" (ptr int)
  let mjModel_geom_matid = field _mjModel "geom_matid" (ptr int)
  let mjModel_geom_group = field _mjModel "geom_group" (ptr int)
  let mjModel_geom_priority = field _mjModel "geom_priority" (ptr int)
  let mjModel_geom_sameframe = field _mjModel "geom_sameframe" (ptr mjtByte)
  let mjModel_geom_solmix = field _mjModel "geom_solmix" (ptr mjtNum)
  let mjModel_geom_solref = field _mjModel "geom_solref" (ptr mjtNum)
  let mjModel_geom_solimp = field _mjModel "geom_solimp" (ptr mjtNum)
  let mjModel_geom_size = field _mjModel "geom_size" (ptr mjtNum)
  let mjModel_geom_rbound = field _mjModel "geom_rbound" (ptr mjtNum)
  let mjModel_geom_pos = field _mjModel "geom_pos" (ptr mjtNum)
  let mjModel_geom_quat = field _mjModel "geom_quat" (ptr mjtNum)
  let mjModel_geom_friction = field _mjModel "geom_friction" (ptr mjtNum)
  let mjModel_geom_margin = field _mjModel "geom_margin" (ptr mjtNum)
  let mjModel_geom_gap = field _mjModel "geom_gap" (ptr mjtNum)
  let mjModel_geom_user = field _mjModel "geom_user" (ptr mjtNum)
  let mjModel_geom_rgba = field _mjModel "geom_rgba" (ptr float)
  let mjModel_site_type = field _mjModel "site_type" (ptr int)
  let mjModel_site_bodyid = field _mjModel "site_bodyid" (ptr int)
  let mjModel_site_matid = field _mjModel "site_matid" (ptr int)
  let mjModel_site_group = field _mjModel "site_group" (ptr int)
  let mjModel_site_sameframe = field _mjModel "site_sameframe" (ptr mjtByte)
  let mjModel_site_size = field _mjModel "site_size" (ptr mjtNum)
  let mjModel_site_pos = field _mjModel "site_pos" (ptr mjtNum)
  let mjModel_site_quat = field _mjModel "site_quat" (ptr mjtNum)
  let mjModel_site_user = field _mjModel "site_user" (ptr mjtNum)
  let mjModel_site_rgba = field _mjModel "site_rgba" (ptr float)
  let mjModel_cam_mode = field _mjModel "cam_mode" (ptr int)
  let mjModel_cam_bodyid = field _mjModel "cam_bodyid" (ptr int)
  let mjModel_cam_targetbodyid = field _mjModel "cam_targetbodyid" (ptr int)
  let mjModel_cam_pos = field _mjModel "cam_pos" (ptr mjtNum)
  let mjModel_cam_quat = field _mjModel "cam_quat" (ptr mjtNum)
  let mjModel_cam_poscom0 = field _mjModel "cam_poscom0" (ptr mjtNum)
  let mjModel_cam_pos0 = field _mjModel "cam_pos0" (ptr mjtNum)
  let mjModel_cam_mat0 = field _mjModel "cam_mat0" (ptr mjtNum)
  let mjModel_cam_fovy = field _mjModel "cam_fovy" (ptr mjtNum)
  let mjModel_cam_ipd = field _mjModel "cam_ipd" (ptr mjtNum)
  let mjModel_cam_user = field _mjModel "cam_user" (ptr mjtNum)
  let mjModel_light_mode = field _mjModel "light_mode" (ptr int)
  let mjModel_light_bodyid = field _mjModel "light_bodyid" (ptr int)
  let mjModel_light_targetbodyid = field _mjModel "light_targetbodyid" (ptr int)
  let mjModel_light_directional = field _mjModel "light_directional" (ptr mjtByte)
  let mjModel_light_castshadow = field _mjModel "light_castshadow" (ptr mjtByte)
  let mjModel_light_active = field _mjModel "light_active" (ptr mjtByte)
  let mjModel_light_pos = field _mjModel "light_pos" (ptr mjtNum)
  let mjModel_light_dir = field _mjModel "light_dir" (ptr mjtNum)
  let mjModel_light_poscom0 = field _mjModel "light_poscom0" (ptr mjtNum)
  let mjModel_light_pos0 = field _mjModel "light_pos0" (ptr mjtNum)
  let mjModel_light_dir0 = field _mjModel "light_dir0" (ptr mjtNum)
  let mjModel_light_attenuation = field _mjModel "light_attenuation" (ptr float)
  let mjModel_light_cutoff = field _mjModel "light_cutoff" (ptr float)
  let mjModel_light_exponent = field _mjModel "light_exponent" (ptr float)
  let mjModel_light_ambient = field _mjModel "light_ambient" (ptr float)
  let mjModel_light_diffuse = field _mjModel "light_diffuse" (ptr float)
  let mjModel_light_specular = field _mjModel "light_specular" (ptr float)
  let mjModel_mesh_vertadr = field _mjModel "mesh_vertadr" (ptr int)
  let mjModel_mesh_vertnum = field _mjModel "mesh_vertnum" (ptr int)
  let mjModel_mesh_texcoordadr = field _mjModel "mesh_texcoordadr" (ptr int)
  let mjModel_mesh_faceadr = field _mjModel "mesh_faceadr" (ptr int)
  let mjModel_mesh_facenum = field _mjModel "mesh_facenum" (ptr int)
  let mjModel_mesh_graphadr = field _mjModel "mesh_graphadr" (ptr int)
  let mjModel_mesh_vert = field _mjModel "mesh_vert" (ptr float)
  let mjModel_mesh_normal = field _mjModel "mesh_normal" (ptr float)
  let mjModel_mesh_texcoord = field _mjModel "mesh_texcoord" (ptr float)
  let mjModel_mesh_face = field _mjModel "mesh_face" (ptr int)
  let mjModel_mesh_graph = field _mjModel "mesh_graph" (ptr int)
  let mjModel_skin_matid = field _mjModel "skin_matid" (ptr int)
  let mjModel_skin_rgba = field _mjModel "skin_rgba" (ptr float)
  let mjModel_skin_inflate = field _mjModel "skin_inflate" (ptr float)
  let mjModel_skin_vertadr = field _mjModel "skin_vertadr" (ptr int)
  let mjModel_skin_vertnum = field _mjModel "skin_vertnum" (ptr int)
  let mjModel_skin_texcoordadr = field _mjModel "skin_texcoordadr" (ptr int)
  let mjModel_skin_faceadr = field _mjModel "skin_faceadr" (ptr int)
  let mjModel_skin_facenum = field _mjModel "skin_facenum" (ptr int)
  let mjModel_skin_boneadr = field _mjModel "skin_boneadr" (ptr int)
  let mjModel_skin_bonenum = field _mjModel "skin_bonenum" (ptr int)
  let mjModel_skin_vert = field _mjModel "skin_vert" (ptr float)
  let mjModel_skin_texcoord = field _mjModel "skin_texcoord" (ptr float)
  let mjModel_skin_face = field _mjModel "skin_face" (ptr int)
  let mjModel_skin_bonevertadr = field _mjModel "skin_bonevertadr" (ptr int)
  let mjModel_skin_bonevertnum = field _mjModel "skin_bonevertnum" (ptr int)
  let mjModel_skin_bonebindpos = field _mjModel "skin_bonebindpos" (ptr float)
  let mjModel_skin_bonebindquat = field _mjModel "skin_bonebindquat" (ptr float)
  let mjModel_skin_bonebodyid = field _mjModel "skin_bonebodyid" (ptr int)
  let mjModel_skin_bonevertid = field _mjModel "skin_bonevertid" (ptr int)
  let mjModel_skin_bonevertweight = field _mjModel "skin_bonevertweight" (ptr float)
  let mjModel_hfield_size = field _mjModel "hfield_size" (ptr mjtNum)
  let mjModel_hfield_nrow = field _mjModel "hfield_nrow" (ptr int)
  let mjModel_hfield_ncol = field _mjModel "hfield_ncol" (ptr int)
  let mjModel_hfield_adr = field _mjModel "hfield_adr" (ptr int)
  let mjModel_hfield_data = field _mjModel "hfield_data" (ptr float)
  let mjModel_tex_type = field _mjModel "tex_type" (ptr int)
  let mjModel_tex_height = field _mjModel "tex_height" (ptr int)
  let mjModel_tex_width = field _mjModel "tex_width" (ptr int)
  let mjModel_tex_adr = field _mjModel "tex_adr" (ptr int)
  let mjModel_tex_rgb = field _mjModel "tex_rgb" (ptr mjtByte)
  let mjModel_mat_texid = field _mjModel "mat_texid" (ptr int)
  let mjModel_mat_texuniform = field _mjModel "mat_texuniform" (ptr mjtByte)
  let mjModel_mat_texrepeat = field _mjModel "mat_texrepeat" (ptr float)
  let mjModel_mat_emission = field _mjModel "mat_emission" (ptr float)
  let mjModel_mat_specular = field _mjModel "mat_specular" (ptr float)
  let mjModel_mat_shininess = field _mjModel "mat_shininess" (ptr float)
  let mjModel_mat_reflectance = field _mjModel "mat_reflectance" (ptr float)
  let mjModel_mat_rgba = field _mjModel "mat_rgba" (ptr float)
  let mjModel_pair_dim = field _mjModel "pair_dim" (ptr int)
  let mjModel_pair_geom1 = field _mjModel "pair_geom1" (ptr int)
  let mjModel_pair_geom2 = field _mjModel "pair_geom2" (ptr int)
  let mjModel_pair_signature = field _mjModel "pair_signature" (ptr int)
  let mjModel_pair_solref = field _mjModel "pair_solref" (ptr mjtNum)
  let mjModel_pair_solimp = field _mjModel "pair_solimp" (ptr mjtNum)
  let mjModel_pair_margin = field _mjModel "pair_margin" (ptr mjtNum)
  let mjModel_pair_gap = field _mjModel "pair_gap" (ptr mjtNum)
  let mjModel_pair_friction = field _mjModel "pair_friction" (ptr mjtNum)
  let mjModel_exclude_signature = field _mjModel "exclude_signature" (ptr int)
  let mjModel_eq_type = field _mjModel "eq_type" (ptr int)
  let mjModel_eq_obj1id = field _mjModel "eq_obj1id" (ptr int)
  let mjModel_eq_obj2id = field _mjModel "eq_obj2id" (ptr int)
  let mjModel_eq_active = field _mjModel "eq_active" (ptr mjtByte)
  let mjModel_eq_solref = field _mjModel "eq_solref" (ptr mjtNum)
  let mjModel_eq_solimp = field _mjModel "eq_solimp" (ptr mjtNum)
  let mjModel_eq_data = field _mjModel "eq_data" (ptr mjtNum)
  let mjModel_tendon_adr = field _mjModel "tendon_adr" (ptr int)
  let mjModel_tendon_num = field _mjModel "tendon_num" (ptr int)
  let mjModel_tendon_matid = field _mjModel "tendon_matid" (ptr int)
  let mjModel_tendon_group = field _mjModel "tendon_group" (ptr int)
  let mjModel_tendon_limited = field _mjModel "tendon_limited" (ptr mjtByte)
  let mjModel_tendon_width = field _mjModel "tendon_width" (ptr mjtNum)
  let mjModel_tendon_solref_lim = field _mjModel "tendon_solref_lim" (ptr mjtNum)
  let mjModel_tendon_solimp_lim = field _mjModel "tendon_solimp_lim" (ptr mjtNum)
  let mjModel_tendon_solref_fri = field _mjModel "tendon_solref_fri" (ptr mjtNum)
  let mjModel_tendon_solimp_fri = field _mjModel "tendon_solimp_fri" (ptr mjtNum)
  let mjModel_tendon_range = field _mjModel "tendon_range" (ptr mjtNum)
  let mjModel_tendon_margin = field _mjModel "tendon_margin" (ptr mjtNum)
  let mjModel_tendon_stiffness = field _mjModel "tendon_stiffness" (ptr mjtNum)
  let mjModel_tendon_damping = field _mjModel "tendon_damping" (ptr mjtNum)
  let mjModel_tendon_frictionloss = field _mjModel "tendon_frictionloss" (ptr mjtNum)
  let mjModel_tendon_lengthspring = field _mjModel "tendon_lengthspring" (ptr mjtNum)
  let mjModel_tendon_length0 = field _mjModel "tendon_length0" (ptr mjtNum)
  let mjModel_tendon_invweight0 = field _mjModel "tendon_invweight0" (ptr mjtNum)
  let mjModel_tendon_user = field _mjModel "tendon_user" (ptr mjtNum)
  let mjModel_tendon_rgba = field _mjModel "tendon_rgba" (ptr float)
  let mjModel_wrap_type = field _mjModel "wrap_type" (ptr int)
  let mjModel_wrap_objid = field _mjModel "wrap_objid" (ptr int)
  let mjModel_wrap_prm = field _mjModel "wrap_prm" (ptr mjtNum)
  let mjModel_actuator_trntype = field _mjModel "actuator_trntype" (ptr int)
  let mjModel_actuator_dyntype = field _mjModel "actuator_dyntype" (ptr int)
  let mjModel_actuator_gaintype = field _mjModel "actuator_gaintype" (ptr int)
  let mjModel_actuator_biastype = field _mjModel "actuator_biastype" (ptr int)
  let mjModel_actuator_trnid = field _mjModel "actuator_trnid" (ptr int)
  let mjModel_actuator_group = field _mjModel "actuator_group" (ptr int)
  let mjModel_actuator_ctrllimited = field _mjModel "actuator_ctrllimited" (ptr mjtByte)
  let mjModel_actuator_forcelimited = field _mjModel "actuator_forcelimited" (ptr mjtByte)
  let mjModel_actuator_dynprm = field _mjModel "actuator_dynprm" (ptr mjtNum)
  let mjModel_actuator_gainprm = field _mjModel "actuator_gainprm" (ptr mjtNum)
  let mjModel_actuator_biasprm = field _mjModel "actuator_biasprm" (ptr mjtNum)
  let mjModel_actuator_ctrlrange = field _mjModel "actuator_ctrlrange" (ptr mjtNum)
  let mjModel_actuator_forcerange = field _mjModel "actuator_forcerange" (ptr mjtNum)
  let mjModel_actuator_gear = field _mjModel "actuator_gear" (ptr mjtNum)
  let mjModel_actuator_cranklength = field _mjModel "actuator_cranklength" (ptr mjtNum)
  let mjModel_actuator_acc0 = field _mjModel "actuator_acc0" (ptr mjtNum)
  let mjModel_actuator_length0 = field _mjModel "actuator_length0" (ptr mjtNum)
  let mjModel_actuator_lengthrange = field _mjModel "actuator_lengthrange" (ptr mjtNum)
  let mjModel_actuator_user = field _mjModel "actuator_user" (ptr mjtNum)
  let mjModel_sensor_type = field _mjModel "sensor_type" (ptr int)
  let mjModel_sensor_datatype = field _mjModel "sensor_datatype" (ptr int)
  let mjModel_sensor_needstage = field _mjModel "sensor_needstage" (ptr int)
  let mjModel_sensor_objtype = field _mjModel "sensor_objtype" (ptr int)
  let mjModel_sensor_objid = field _mjModel "sensor_objid" (ptr int)
  let mjModel_sensor_dim = field _mjModel "sensor_dim" (ptr int)
  let mjModel_sensor_adr = field _mjModel "sensor_adr" (ptr int)
  let mjModel_sensor_cutoff = field _mjModel "sensor_cutoff" (ptr mjtNum)
  let mjModel_sensor_noise = field _mjModel "sensor_noise" (ptr mjtNum)
  let mjModel_sensor_user = field _mjModel "sensor_user" (ptr mjtNum)
  let mjModel_numeric_adr = field _mjModel "numeric_adr" (ptr int)
  let mjModel_numeric_size = field _mjModel "numeric_size" (ptr int)
  let mjModel_numeric_data = field _mjModel "numeric_data" (ptr mjtNum)
  let mjModel_text_adr = field _mjModel "text_adr" (ptr int)
  let mjModel_text_size = field _mjModel "text_size" (ptr int)
  let mjModel_text_data = field _mjModel "text_data" string
  let mjModel_tuple_adr = field _mjModel "tuple_adr" (ptr int)
  let mjModel_tuple_size = field _mjModel "tuple_size" (ptr int)
  let mjModel_tuple_objtype = field _mjModel "tuple_objtype" (ptr int)
  let mjModel_tuple_objid = field _mjModel "tuple_objid" (ptr int)
  let mjModel_tuple_objprm = field _mjModel "tuple_objprm" (ptr mjtNum)
  let mjModel_key_time = field _mjModel "key_time" (ptr mjtNum)
  let mjModel_key_qpos = field _mjModel "key_qpos" (ptr mjtNum)
  let mjModel_key_qvel = field _mjModel "key_qvel" (ptr mjtNum)
  let mjModel_key_act = field _mjModel "key_act" (ptr mjtNum)
  let mjModel_key_mpos = field _mjModel "key_mpos" (ptr mjtNum)
  let mjModel_key_mquat = field _mjModel "key_mquat" (ptr mjtNum)
  let mjModel_name_bodyadr = field _mjModel "name_bodyadr" (ptr int)
  let mjModel_name_jntadr = field _mjModel "name_jntadr" (ptr int)
  let mjModel_name_geomadr = field _mjModel "name_geomadr" (ptr int)
  let mjModel_name_siteadr = field _mjModel "name_siteadr" (ptr int)
  let mjModel_name_camadr = field _mjModel "name_camadr" (ptr int)
  let mjModel_name_lightadr = field _mjModel "name_lightadr" (ptr int)
  let mjModel_name_meshadr = field _mjModel "name_meshadr" (ptr int)
  let mjModel_name_skinadr = field _mjModel "name_skinadr" (ptr int)
  let mjModel_name_hfieldadr = field _mjModel "name_hfieldadr" (ptr int)
  let mjModel_name_texadr = field _mjModel "name_texadr" (ptr int)
  let mjModel_name_matadr = field _mjModel "name_matadr" (ptr int)
  let mjModel_name_pairadr = field _mjModel "name_pairadr" (ptr int)
  let mjModel_name_excludeadr = field _mjModel "name_excludeadr" (ptr int)
  let mjModel_name_eqadr = field _mjModel "name_eqadr" (ptr int)
  let mjModel_name_tendonadr = field _mjModel "name_tendonadr" (ptr int)
  let mjModel_name_actuatoradr = field _mjModel "name_actuatoradr" (ptr int)
  let mjModel_name_sensoradr = field _mjModel "name_sensoradr" (ptr int)
  let mjModel_name_numericadr = field _mjModel "name_numericadr" (ptr int)
  let mjModel_name_textadr = field _mjModel "name_textadr" (ptr int)
  let mjModel_name_tupleadr = field _mjModel "name_tupleadr" (ptr int)
  let mjModel_name_keyadr = field _mjModel "name_keyadr" (ptr int)
  let mjModel_names = field _mjModel "names" string
  let () = seal _mjModel

  type mjModel = _mjModel

  let mjModel = _mjModel

  (* ------------------------- mjdata.h ------------------------- *)

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
  let mjContact_dist = field _mjContact "dist" mjtNum
  let mjContact_includemargin = field _mjContact "includemargin" mjtNum
  let mjContact_mu = field _mjContact "mu" mjtNum
  let mjContact_dim = field _mjContact "dim" int
  let mjContact_geom1 = field _mjContact "geom1" int
  let mjContact_geom2 = field _mjContact "geom2" int
  let mjContact_exclude = field _mjContact "exclude" int
  let mjContact_efc_address = field _mjContact "efc_address" int
  let () = seal _mjContact

  type mjContact = _mjContact

  let mjContact = _mjContact

  type _mjWarningStat

  let _mjWarningStat : _mjWarningStat structure typ = structure "_mjWarningStat"
  let mjWarningStat_lastinfo = field _mjWarningStat "lastinfo" int
  let mjWarningStat_number = field _mjWarningStat "number" int
  let () = seal _mjWarningStat

  type mjWarningStat = _mjWarningStat

  let mjWarningStat = _mjWarningStat

  type _mjTimerStat

  let _mjTimerStat : _mjTimerStat structure typ = structure "_mjTimerStat"
  let mjTimerStat_duration = field _mjTimerStat "duration" mjtNum
  let mjTimerStat_number = field _mjTimerStat "number" int
  let () = seal _mjTimerStat

  type mjTimerStat = _mjTimerStat

  let mjTimerStat = _mjTimerStat

  type _mjSolverStat

  let _mjSolverStat : _mjSolverStat structure typ = structure "_mjSolverStat"
  let mjSolverStat_improvement = field _mjSolverStat "improvement" mjtNum
  let mjSolverStat_gradient = field _mjSolverStat "gradient" mjtNum
  let mjSolverStat_lineslope = field _mjSolverStat "lineslope" mjtNum
  let mjSolverStat_nactive = field _mjSolverStat "nactive" int
  let mjSolverStat_nchange = field _mjSolverStat "nchange" int
  let mjSolverStat_neval = field _mjSolverStat "neval" int
  let mjSolverStat_nupdate = field _mjSolverStat "nupdate" int
  let () = seal _mjSolverStat

  type mjSolverStat = _mjSolverStat

  let mjSolverStat = _mjSolverStat

  type _mjData

  let _mjData : _mjData structure typ = structure "_mjData"
  let mjData_nstack = field _mjData "nstack" int
  let mjData_nbuffer = field _mjData "nbuffer" int
  let mjData_pstack = field _mjData "pstack" int
  let mjData_maxuse_stack = field _mjData "maxuse_stack" int
  let mjData_maxuse_con = field _mjData "maxuse_con" int
  let mjData_maxuse_efc = field _mjData "maxuse_efc" int
  let mjData_solver_iter = field _mjData "solver_iter" int
  let mjData_solver_nnz = field _mjData "solver_nnz" int
  let mjData_ne = field _mjData "ne" int
  let mjData_nf = field _mjData "nf" int
  let mjData_nefc = field _mjData "nefc" int
  let mjData_ncon = field _mjData "ncon" int
  let mjData_time = field _mjData "time" mjtNum
  let mjData_buffer = field _mjData "buffer" (ptr void)
  let mjData_stack = field _mjData "stack" (ptr mjtNum)
  let mjData_qpos = field _mjData "qpos" (ptr mjtNum)
  let mjData_qvel = field _mjData "qvel" (ptr mjtNum)
  let mjData_act = field _mjData "act" (ptr mjtNum)
  let mjData_qacc_warmstart = field _mjData "qacc_warmstart" (ptr mjtNum)
  let mjData_ctrl = field _mjData "ctrl" (ptr mjtNum)
  let mjData_qfrc_applied = field _mjData "qfrc_applied" (ptr mjtNum)
  let mjData_xfrc_applied = field _mjData "xfrc_applied" (ptr mjtNum)
  let mjData_qacc = field _mjData "qacc" (ptr mjtNum)
  let mjData_act_dot = field _mjData "act_dot" (ptr mjtNum)
  let mjData_mocap_pos = field _mjData "mocap_pos" (ptr mjtNum)
  let mjData_mocap_quat = field _mjData "mocap_quat" (ptr mjtNum)
  let mjData_userdata = field _mjData "userdata" (ptr mjtNum)
  let mjData_sensordata = field _mjData "sensordata" (ptr mjtNum)
  let mjData_xpos = field _mjData "xpos" (ptr mjtNum)
  let mjData_xquat = field _mjData "xquat" (ptr mjtNum)
  let mjData_xmat = field _mjData "xmat" (ptr mjtNum)
  let mjData_xipos = field _mjData "xipos" (ptr mjtNum)
  let mjData_ximat = field _mjData "ximat" (ptr mjtNum)
  let mjData_xanchor = field _mjData "xanchor" (ptr mjtNum)
  let mjData_xaxis = field _mjData "xaxis" (ptr mjtNum)
  let mjData_geom_xpos = field _mjData "geom_xpos" (ptr mjtNum)
  let mjData_geom_xmat = field _mjData "geom_xmat" (ptr mjtNum)
  let mjData_site_xpos = field _mjData "site_xpos" (ptr mjtNum)
  let mjData_site_xmat = field _mjData "site_xmat" (ptr mjtNum)
  let mjData_cam_xpos = field _mjData "cam_xpos" (ptr mjtNum)
  let mjData_cam_xmat = field _mjData "cam_xmat" (ptr mjtNum)
  let mjData_light_xpos = field _mjData "light_xpos" (ptr mjtNum)
  let mjData_light_xdir = field _mjData "light_xdir" (ptr mjtNum)
  let mjData_subtree_com = field _mjData "subtree_com" (ptr mjtNum)
  let mjData_cdof = field _mjData "cdof" (ptr mjtNum)
  let mjData_cinert = field _mjData "cinert" (ptr mjtNum)
  let mjData_ten_wrapadr = field _mjData "ten_wrapadr" (ptr int)
  let mjData_ten_wrapnum = field _mjData "ten_wrapnum" (ptr int)
  let mjData_ten_J_rownnz = field _mjData "ten_J_rownnz" (ptr int)
  let mjData_ten_J_rowadr = field _mjData "ten_J_rowadr" (ptr int)
  let mjData_ten_J_colind = field _mjData "ten_J_colind" (ptr int)
  let mjData_ten_length = field _mjData "ten_length" (ptr mjtNum)
  let mjData_ten_J = field _mjData "ten_J" (ptr mjtNum)
  let mjData_wrap_obj = field _mjData "wrap_obj" (ptr int)
  let mjData_wrap_xpos = field _mjData "wrap_xpos" (ptr mjtNum)
  let mjData_actuator_length = field _mjData "actuator_length" (ptr mjtNum)
  let mjData_actuator_moment = field _mjData "actuator_moment" (ptr mjtNum)
  let mjData_crb = field _mjData "crb" (ptr mjtNum)
  let mjData_qM = field _mjData "qM" (ptr mjtNum)
  let mjData_qLD = field _mjData "qLD" (ptr mjtNum)
  let mjData_qLDiagInv = field _mjData "qLDiagInv" (ptr mjtNum)
  let mjData_qLDiagSqrtInv = field _mjData "qLDiagSqrtInv" (ptr mjtNum)
  let mjData_contact = field _mjData "contact" (ptr mjContact)
  let mjData_efc_type = field _mjData "efc_type" (ptr int)
  let mjData_efc_id = field _mjData "efc_id" (ptr int)
  let mjData_efc_J_rownnz = field _mjData "efc_J_rownnz" (ptr int)
  let mjData_efc_J_rowadr = field _mjData "efc_J_rowadr" (ptr int)
  let mjData_efc_J_rowsuper = field _mjData "efc_J_rowsuper" (ptr int)
  let mjData_efc_J_colind = field _mjData "efc_J_colind" (ptr int)
  let mjData_efc_JT_rownnz = field _mjData "efc_JT_rownnz" (ptr int)
  let mjData_efc_JT_rowadr = field _mjData "efc_JT_rowadr" (ptr int)
  let mjData_efc_JT_rowsuper = field _mjData "efc_JT_rowsuper" (ptr int)
  let mjData_efc_JT_colind = field _mjData "efc_JT_colind" (ptr int)
  let mjData_efc_J = field _mjData "efc_J" (ptr mjtNum)
  let mjData_efc_JT = field _mjData "efc_JT" (ptr mjtNum)
  let mjData_efc_pos = field _mjData "efc_pos" (ptr mjtNum)
  let mjData_efc_margin = field _mjData "efc_margin" (ptr mjtNum)
  let mjData_efc_frictionloss = field _mjData "efc_frictionloss" (ptr mjtNum)
  let mjData_efc_diagApprox = field _mjData "efc_diagApprox" (ptr mjtNum)
  let mjData_efc_KBIP = field _mjData "efc_KBIP" (ptr mjtNum)
  let mjData_efc_D = field _mjData "efc_D" (ptr mjtNum)
  let mjData_efc_R = field _mjData "efc_R" (ptr mjtNum)
  let mjData_efc_AR_rownnz = field _mjData "efc_AR_rownnz" (ptr int)
  let mjData_efc_AR_rowadr = field _mjData "efc_AR_rowadr" (ptr int)
  let mjData_efc_AR_colind = field _mjData "efc_AR_colind" (ptr int)
  let mjData_efc_AR = field _mjData "efc_AR" (ptr mjtNum)
  let mjData_ten_velocity = field _mjData "ten_velocity" (ptr mjtNum)
  let mjData_actuator_velocity = field _mjData "actuator_velocity" (ptr mjtNum)
  let mjData_cvel = field _mjData "cvel" (ptr mjtNum)
  let mjData_cdof_dot = field _mjData "cdof_dot" (ptr mjtNum)
  let mjData_qfrc_bias = field _mjData "qfrc_bias" (ptr mjtNum)
  let mjData_qfrc_passive = field _mjData "qfrc_passive" (ptr mjtNum)
  let mjData_efc_vel = field _mjData "efc_vel" (ptr mjtNum)
  let mjData_efc_aref = field _mjData "efc_aref" (ptr mjtNum)
  let mjData_subtree_linvel = field _mjData "subtree_linvel" (ptr mjtNum)
  let mjData_subtree_angmom = field _mjData "subtree_angmom" (ptr mjtNum)
  let mjData_actuator_force = field _mjData "actuator_force" (ptr mjtNum)
  let mjData_qfrc_actuator = field _mjData "qfrc_actuator" (ptr mjtNum)
  let mjData_qfrc_unc = field _mjData "qfrc_unc" (ptr mjtNum)
  let mjData_qacc_unc = field _mjData "qacc_unc" (ptr mjtNum)
  let mjData_efc_b = field _mjData "efc_b" (ptr mjtNum)
  let mjData_efc_force = field _mjData "efc_force" (ptr mjtNum)
  let mjData_efc_state = field _mjData "efc_state" (ptr int)
  let mjData_qfrc_constraint = field _mjData "qfrc_constraint" (ptr mjtNum)
  let mjData_qfrc_inverse = field _mjData "qfrc_inverse" (ptr mjtNum)
  let mjData_cacc = field _mjData "cacc" (ptr mjtNum)
  let mjData_cfrc_int = field _mjData "cfrc_int" (ptr mjtNum)
  let mjData_cfrc_ext = field _mjData "cfrc_ext" (ptr mjtNum)
  let () = seal _mjData

  type mjData = _mjData

  let mjData = _mjData

  (* ------------------------- mjvisualize.h ------------------------- *)

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
  let mjvPerturb_select = field _mjvPerturb "select" int
  let mjvPerturb_skinselect = field _mjvPerturb "skinselect" int
  let mjvPerturb_active = field _mjvPerturb "active" int
  let mjvPerturb_active2 = field _mjvPerturb "active2" int
  let mjvPerturb_scale = field _mjvPerturb "scale" mjtNum
  let () = seal _mjvPerturb

  type mjvPerturb = _mjvPerturb

  let mjvPerturb = _mjvPerturb

  type _mjvCamera

  let _mjvCamera : _mjvCamera structure typ = structure "_mjvCamera"
  let mjvCamera_type = field _mjvCamera "type" int
  let mjvCamera_fixedcamid = field _mjvCamera "fixedcamid" int
  let mjvCamera_trackbodyid = field _mjvCamera "trackbodyid" int
  let mjvCamera_distance = field _mjvCamera "distance" mjtNum
  let mjvCamera_azimuth = field _mjvCamera "azimuth" mjtNum
  let mjvCamera_elevation = field _mjvCamera "elevation" mjtNum
  let () = seal _mjvCamera

  type mjvCamera = _mjvCamera

  let mjvCamera = _mjvCamera

  type _mjvGLCamera

  let _mjvGLCamera : _mjvGLCamera structure typ = structure "_mjvGLCamera"
  let mjvGLCamera_frustum_center = field _mjvGLCamera "frustum_center" float
  let mjvGLCamera_frustum_bottom = field _mjvGLCamera "frustum_bottom" float
  let mjvGLCamera_frustum_top = field _mjvGLCamera "frustum_top" float
  let mjvGLCamera_frustum_near = field _mjvGLCamera "frustum_near" float
  let mjvGLCamera_frustum_far = field _mjvGLCamera "frustum_far" float
  let () = seal _mjvGLCamera

  type mjvGLCamera = _mjvGLCamera

  let mjvGLCamera = _mjvGLCamera

  type _mjvGeom

  let _mjvGeom : _mjvGeom structure typ = structure "_mjvGeom"
  let mjvGeom_type = field _mjvGeom "type" int
  let mjvGeom_dataid = field _mjvGeom "dataid" int
  let mjvGeom_objtype = field _mjvGeom "objtype" int
  let mjvGeom_objid = field _mjvGeom "objid" int
  let mjvGeom_category = field _mjvGeom "category" int
  let mjvGeom_texid = field _mjvGeom "texid" int
  let mjvGeom_texuniform = field _mjvGeom "texuniform" int
  let mjvGeom_texcoord = field _mjvGeom "texcoord" int
  let mjvGeom_segid = field _mjvGeom "segid" int
  let mjvGeom_emission = field _mjvGeom "emission" float
  let mjvGeom_specular = field _mjvGeom "specular" float
  let mjvGeom_shininess = field _mjvGeom "shininess" float
  let mjvGeom_reflectance = field _mjvGeom "reflectance" float
  let mjvGeom_camdist = field _mjvGeom "camdist" float
  let mjvGeom_modelrbound = field _mjvGeom "modelrbound" float
  let mjvGeom_transparent = field _mjvGeom "transparent" mjtByte
  let () = seal _mjvGeom

  type mjvGeom = _mjvGeom

  let mjvGeom = _mjvGeom

  type _mjvLight

  let _mjvLight : _mjvLight structure typ = structure "_mjvLight"
  let mjvLight_cutoff = field _mjvLight "cutoff" float
  let mjvLight_exponent = field _mjvLight "exponent" float
  let mjvLight_headlight = field _mjvLight "headlight" mjtByte
  let mjvLight_directional = field _mjvLight "directional" mjtByte
  let mjvLight_castshadow = field _mjvLight "castshadow" mjtByte
  let () = seal _mjvLight

  type mjvLight = _mjvLight

  let mjvLight = _mjvLight

  type _mjvOption

  let _mjvOption : _mjvOption structure typ = structure "_mjvOption"
  let mjvOption_label = field _mjvOption "label" int
  let mjvOption_frame = field _mjvOption "frame" int
  let () = seal _mjvOption

  type mjvOption = _mjvOption

  let mjvOption = _mjvOption

  type _mjvScene

  let _mjvScene : _mjvScene structure typ = structure "_mjvScene"
  let mjvScene_maxgeom = field _mjvScene "maxgeom" int
  let mjvScene_ngeom = field _mjvScene "ngeom" int
  let mjvScene_geoms = field _mjvScene "geoms" (ptr mjvGeom)
  let mjvScene_geomorder = field _mjvScene "geomorder" (ptr int)
  let mjvScene_nskin = field _mjvScene "nskin" int
  let mjvScene_skinfacenum = field _mjvScene "skinfacenum" (ptr int)
  let mjvScene_skinvertadr = field _mjvScene "skinvertadr" (ptr int)
  let mjvScene_skinvertnum = field _mjvScene "skinvertnum" (ptr int)
  let mjvScene_skinvert = field _mjvScene "skinvert" (ptr float)
  let mjvScene_skinnormal = field _mjvScene "skinnormal" (ptr float)
  let mjvScene_nlight = field _mjvScene "nlight" int
  let mjvScene_enabletransform = field _mjvScene "enabletransform" mjtByte
  let mjvScene_scale = field _mjvScene "scale" float
  let mjvScene_stereo = field _mjvScene "stereo" int
  let mjvScene_framewidth = field _mjvScene "framewidth" int
  let () = seal _mjvScene

  type mjvScene = _mjvScene

  let mjvScene = _mjvScene

  type _mjvFigure

  let _mjvFigure : _mjvFigure structure typ = structure "_mjvFigure"
  let mjvFigure_flg_legend = field _mjvFigure "flg_legend" int
  let mjvFigure_flg_extend = field _mjvFigure "flg_extend" int
  let mjvFigure_flg_barplot = field _mjvFigure "flg_barplot" int
  let mjvFigure_flg_selection = field _mjvFigure "flg_selection" int
  let mjvFigure_flg_symmetric = field _mjvFigure "flg_symmetric" int
  let mjvFigure_linewidth = field _mjvFigure "linewidth" float
  let mjvFigure_gridwidth = field _mjvFigure "gridwidth" float
  let mjvFigure_legendoffset = field _mjvFigure "legendoffset" int
  let mjvFigure_subplot = field _mjvFigure "subplot" int
  let mjvFigure_highlightid = field _mjvFigure "highlightid" int
  let mjvFigure_selection = field _mjvFigure "selection" float
  let () = seal _mjvFigure

  type mjvFigure = _mjvFigure

  let mjvFigure = _mjvFigure

  (* ------------------------- mjrender.h ------------------------- *)

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
  let mjrRect_left = field _mjrRect "left" int
  let mjrRect_bottom = field _mjrRect "bottom" int
  let mjrRect_width = field _mjrRect "width" int
  let mjrRect_height = field _mjrRect "height" int
  let () = seal _mjrRect

  type mjrRect = _mjrRect

  let mjrRect = _mjrRect

  type _mjrContext

  let _mjrContext : _mjrContext structure typ = structure "_mjrContext"
  let mjrContext_lineWidth = field _mjrContext "lineWidth" float
  let mjrContext_shadowClip = field _mjrContext "shadowClip" float
  let mjrContext_shadowScale = field _mjrContext "shadowScale" float
  let mjrContext_fogStart = field _mjrContext "fogStart" float
  let mjrContext_fogEnd = field _mjrContext "fogEnd" float
  let mjrContext_shadowSize = field _mjrContext "shadowSize" int
  let mjrContext_offWidth = field _mjrContext "offWidth" int
  let mjrContext_offHeight = field _mjrContext "offHeight" int
  let mjrContext_offSamples = field _mjrContext "offSamples" int
  let mjrContext_fontScale = field _mjrContext "fontScale" int
  let mjrContext_ntexture = field _mjrContext "ntexture" int
  let mjrContext_rangePlane = field _mjrContext "rangePlane" int
  let mjrContext_rangeMesh = field _mjrContext "rangeMesh" int
  let mjrContext_rangeHField = field _mjrContext "rangeHField" int
  let mjrContext_rangeBuiltin = field _mjrContext "rangeBuiltin" int
  let mjrContext_rangeFont = field _mjrContext "rangeFont" int
  let mjrContext_nskin = field _mjrContext "nskin" int
  let mjrContext_charHeight = field _mjrContext "charHeight" int
  let mjrContext_charHeightBig = field _mjrContext "charHeightBig" int
  let mjrContext_glewInitialized = field _mjrContext "glewInitialized" int
  let mjrContext_windowAvailable = field _mjrContext "windowAvailable" int
  let mjrContext_windowSamples = field _mjrContext "windowSamples" int
  let mjrContext_windowStereo = field _mjrContext "windowStereo" int
  let mjrContext_windowDoublebuffer = field _mjrContext "windowDoublebuffer" int
  let mjrContext_currentBuffer = field _mjrContext "currentBuffer" int
  let () = seal _mjrContext

  type mjrContext = _mjrContext

  let mjrContext = _mjrContext

  (* ------------------------- mjui.h ------------------------- *)

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
  let mjuiState_nrect = field _mjuiState "nrect" int
  let mjuiState_userdata = field _mjuiState "userdata" (ptr void)
  let mjuiState_type = field _mjuiState "type" int
  let mjuiState_left = field _mjuiState "left" int
  let mjuiState_right = field _mjuiState "right" int
  let mjuiState_middle = field _mjuiState "middle" int
  let mjuiState_doubleclick = field _mjuiState "doubleclick" int
  let mjuiState_button = field _mjuiState "button" int
  let mjuiState_buttontime = field _mjuiState "buttontime" double
  let mjuiState_x = field _mjuiState "x" double
  let mjuiState_y = field _mjuiState "y" double
  let mjuiState_dx = field _mjuiState "dx" double
  let mjuiState_dy = field _mjuiState "dy" double
  let mjuiState_sx = field _mjuiState "sx" double
  let mjuiState_sy = field _mjuiState "sy" double
  let mjuiState_control = field _mjuiState "control" int
  let mjuiState_shift = field _mjuiState "shift" int
  let mjuiState_alt = field _mjuiState "alt" int
  let mjuiState_key = field _mjuiState "key" int
  let mjuiState_keytime = field _mjuiState "keytime" double
  let mjuiState_mouserect = field _mjuiState "mouserect" int
  let mjuiState_dragrect = field _mjuiState "dragrect" int
  let mjuiState_dragbutton = field _mjuiState "dragbutton" int
  let () = seal _mjuiState

  type mjuiState = _mjuiState

  let mjuiState = _mjuiState

  type _mjuiThemeSpacing

  let _mjuiThemeSpacing : _mjuiThemeSpacing structure typ = structure "_mjuiThemeSpacing"
  let mjuiThemeSpacing_total = field _mjuiThemeSpacing "total" int
  let mjuiThemeSpacing_scroll = field _mjuiThemeSpacing "scroll" int
  let mjuiThemeSpacing_label = field _mjuiThemeSpacing "label" int
  let mjuiThemeSpacing_section = field _mjuiThemeSpacing "section" int
  let mjuiThemeSpacing_itemside = field _mjuiThemeSpacing "itemside" int
  let mjuiThemeSpacing_itemmid = field _mjuiThemeSpacing "itemmid" int
  let mjuiThemeSpacing_itemver = field _mjuiThemeSpacing "itemver" int
  let mjuiThemeSpacing_texthor = field _mjuiThemeSpacing "texthor" int
  let mjuiThemeSpacing_textver = field _mjuiThemeSpacing "textver" int
  let mjuiThemeSpacing_linescroll = field _mjuiThemeSpacing "linescroll" int
  let mjuiThemeSpacing_samples = field _mjuiThemeSpacing "samples" int
  let () = seal _mjuiThemeSpacing

  type mjuiThemeSpacing = _mjuiThemeSpacing

  let mjuiThemeSpacing = _mjuiThemeSpacing

  type _mjuiThemeColor

  let _mjuiThemeColor : _mjuiThemeColor structure typ = structure "_mjuiThemeColor"
  let () = seal _mjuiThemeColor

  type mjuiThemeColor = _mjuiThemeColor

  let mjuiThemeColor = _mjuiThemeColor

  type _mjuiItemSingle

  let _mjuiItemSingle : _mjuiItemSingle structure typ = structure "_mjuiItemSingle"
  let mjuiItemSingle_modifier = field _mjuiItemSingle "modifier" int
  let mjuiItemSingle_shortcut = field _mjuiItemSingle "shortcut" int
  let () = seal _mjuiItemSingle

  type mjuiItemSingle = _mjuiItemSingle

  let mjuiItemSingle = _mjuiItemSingle

  type _mjuiItemMulti

  let _mjuiItemMulti : _mjuiItemMulti structure typ = structure "_mjuiItemMulti"
  let mjuiItemMulti_nelem = field _mjuiItemMulti "nelem" int
  let () = seal _mjuiItemMulti

  type mjuiItemMulti = _mjuiItemMulti

  let mjuiItemMulti = _mjuiItemMulti

  type _mjuiItemSlider

  let _mjuiItemSlider : _mjuiItemSlider structure typ = structure "_mjuiItemSlider"
  let mjuiItemSlider_divisions = field _mjuiItemSlider "divisions" double
  let () = seal _mjuiItemSlider

  type mjuiItemSlider = _mjuiItemSlider

  let mjuiItemSlider = _mjuiItemSlider

  type _mjuiItemEdit

  let _mjuiItemEdit : _mjuiItemEdit structure typ = structure "_mjuiItemEdit"
  let mjuiItemEdit_nelem = field _mjuiItemEdit "nelem" int
  let () = seal _mjuiItemEdit

  type mjuiItemEdit = _mjuiItemEdit

  let mjuiItemEdit = _mjuiItemEdit

  type _mjuiItem

  let _mjuiItem : _mjuiItem structure typ = structure "_mjuiItem"
  let mjuiItem_type = field _mjuiItem "type" int
  let mjuiItem_state = field _mjuiItem "state" int
  let mjuiItem_sectionid = field _mjuiItem "sectionid" int
  let mjuiItem_itemid = field _mjuiItem "itemid" int
  let mjuiItem_rect = field _mjuiItem "rect" mjrRect
  let () = seal _mjuiItem

  type mjuiItem = _mjuiItem

  let mjuiItem = _mjuiItem

  type _mjuiSection

  let _mjuiSection : _mjuiSection structure typ = structure "_mjuiSection"
  let mjuiSection_state = field _mjuiSection "state" int
  let mjuiSection_modifier = field _mjuiSection "modifier" int
  let mjuiSection_shortcut = field _mjuiSection "shortcut" int
  let mjuiSection_nitem = field _mjuiSection "nitem" int
  let mjuiSection_rtitle = field _mjuiSection "rtitle" mjrRect
  let mjuiSection_rcontent = field _mjuiSection "rcontent" mjrRect
  let () = seal _mjuiSection

  type mjuiSection = _mjuiSection

  let mjuiSection = _mjuiSection

  type _mjUI

  let _mjUI : _mjUI structure typ = structure "_mjUI"
  let mjUI_spacing = field _mjUI "spacing" mjuiThemeSpacing
  let mjUI_color = field _mjUI "color" mjuiThemeColor
  let mjUI_predicate = field _mjUI "predicate" mjfItemEnable
  let mjUI_userdata = field _mjUI "userdata" (ptr void)
  let mjUI_rectid = field _mjUI "rectid" int
  let mjUI_auxid = field _mjUI "auxid" int
  let mjUI_radiocol = field _mjUI "radiocol" int
  let mjUI_width = field _mjUI "width" int
  let mjUI_height = field _mjUI "height" int
  let mjUI_maxheight = field _mjUI "maxheight" int
  let mjUI_scroll = field _mjUI "scroll" int
  let mjUI_mousesect = field _mjUI "mousesect" int
  let mjUI_mouseitem = field _mjUI "mouseitem" int
  let mjUI_mousehelp = field _mjUI "mousehelp" int
  let mjUI_editsect = field _mjUI "editsect" int
  let mjUI_edititem = field _mjUI "edititem" int
  let mjUI_editcursor = field _mjUI "editcursor" int
  let mjUI_editscroll = field _mjUI "editscroll" int
  let mjUI_editchanged = field _mjUI "editchanged" (ptr mjuiItem)
  let mjUI_nsect = field _mjUI "nsect" int
  let () = seal _mjUI

  type mjUI = _mjUI

  let mjUI = _mjUI

  type _mjuiDef

  let _mjuiDef : _mjuiDef structure typ = structure "_mjuiDef"
  let mjuiDef_type = field _mjuiDef "type" int
  let mjuiDef_state = field _mjuiDef "state" int
  let mjuiDef_pdata = field _mjuiDef "pdata" (ptr void)
  let () = seal _mjuiDef

  type mjuiDef = _mjuiDef

  let mjuiDef = _mjuiDef
end
