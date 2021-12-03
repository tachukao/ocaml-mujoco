(* THIS FILE IS GENERATED AUTOMATICALLY, DO NOT EDIT BY HAND *)

module Typs = Stubs.Typs
module Bindings = Stubs.Bindings (Mujoco_generated)

type 'a t = 'a Ctypes.structure
type 'a ptr = 'a Ctypes_static.ptr

(** get value of ptr *)
let ( !@ ) = Ctypes.( !@ )

(** get ptr of struct *)
let ( !& ) = Ctypes.addr

type mjfItemEnable = (int -> unit ptr -> int) Mujoco_generated.CI.static_funptr

(** disable default feature bitflags *)
type mjtDisableBit = Typs.mjtDisableBit =
  | MjDSBL_CONSTRAINT (** entire constraint solver *)
  | MjDSBL_EQUALITY (** equality constraints *)
  | MjDSBL_FRICTIONLOSS (** joint and tendon frictionloss constraints *)
  | MjDSBL_LIMIT (** joint and tendon limit constraints *)
  | MjDSBL_CONTACT (** contact constraints *)
  | MjDSBL_PASSIVE (** passive forces *)
  | MjDSBL_GRAVITY (** gravitational forces *)
  | MjDSBL_CLAMPCTRL (** clamp control to specified range *)
  | MjDSBL_WARMSTART (** warmstart constraint solver *)
  | MjDSBL_FILTERPARENT (** remove collisions with parent body *)
  | MjDSBL_ACTUATION (** apply actuation forces *)
  | MjDSBL_REFSAFE (** integrator safety: make ref[0]>=2*timestep *)
  | MjNDISABLE (** number of disable flags, *)

(** enable optional feature bitflags *)
type mjtEnableBit = Typs.mjtEnableBit =
  | MjENBL_OVERRIDE (** override contact parameters *)
  | MjENBL_ENERGY (** energy computation *)
  | MjENBL_FWDINV (** record solver statistics *)
  | MjENBL_SENSORNOISE (** add noise to sensor data *)
  | MjNENABLE (** number of enable flags, *)

(** type of degree of freedom *)
type mjtJoint = Typs.mjtJoint =
  | MjJNT_FREE (** global position and orientation (quat)       (7) *)
  | MjJNT_BALL (** orientation (quat) relative to parent        (4) *)
  | MjJNT_SLIDE (** sliding distance along body-fixed axis       (1) *)
  | MjJNT_HINGE (** rotation angle (rad) around body-fixed axis  (1), *)

(** type of geometric shape *)
type mjtGeom = Typs.mjtGeom =
  | MjGEOM_PLANE (** plane *)
  | MjGEOM_HFIELD (** height field *)
  | MjGEOM_SPHERE (** sphere *)
  | MjGEOM_CAPSULE (** capsule *)
  | MjGEOM_ELLIPSOID (** ellipsoid *)
  | MjGEOM_CYLINDER (** cylinder *)
  | MjGEOM_BOX (** box *)
  | MjGEOM_MESH (** mesh *)
  | MjNGEOMTYPES (** number of regular geom types *)
  | MjGEOM_ARROW (** arrow *)
  | MjGEOM_ARROW1 (** arrow without wedges *)
  | MjGEOM_ARROW2 (** arrow in both directions *)
  | MjGEOM_LINE (** line *)
  | MjGEOM_SKIN (** skin *)
  | MjGEOM_LABEL (** text label *)
  | MjGEOM_NONE (** missing geom type, *)

(** tracking mode for camera and light *)
type mjtCamLight = Typs.mjtCamLight =
  | MjCAMLIGHT_FIXED (** pos and rot fixed in body *)
  | MjCAMLIGHT_TRACK (** pos tracks body, rot fixed in global *)
  | MjCAMLIGHT_TRACKCOM (** pos tracks subtree com, rot fixed in body *)
  | MjCAMLIGHT_TARGETBODY (** pos fixed in body, rot tracks target body *)
  | MjCAMLIGHT_TARGETBODYCOM (** pos fixed in body, rot tracks target subtree com, *)

(** type of texture *)
type mjtTexture = Typs.mjtTexture =
  | MjTEXTURE_2D (** 2d texture, suitable for planes and hfields *)
  | MjTEXTURE_CUBE (** cube texture, suitable for all other geom types *)
  | MjTEXTURE_SKYBOX (** cube texture used as skybox, *)

(** integrator mode *)
type mjtIntegrator = Typs.mjtIntegrator =
  | MjINT_EULER (** semi-implicit Euler *)
  | MjINT_RK4 (** 4th-order Runge Kutta, *)

(** collision mode for selecting geom pairs *)
type mjtCollision = Typs.mjtCollision =
  | MjCOL_ALL (** test precomputed and dynamic pairs *)
  | MjCOL_PAIR (** test predefined pairs only *)
  | MjCOL_DYNAMIC (** test dynamic pairs only, *)

(** type of friction cone *)
type mjtCone = Typs.mjtCone =
  | MjCONE_PYRAMIDAL (** pyramidal *)
  | MjCONE_ELLIPTIC (** elliptic, *)

(** type of constraint Jacobian *)
type mjtJacobian = Typs.mjtJacobian =
  | MjJAC_DENSE (** dense *)
  | MjJAC_SPARSE (** sparse *)
  | MjJAC_AUTO (** dense if nv<60, sparse otherwise, *)

(** constraint solver algorithm *)
type mjtSolver = Typs.mjtSolver =
  | MjSOL_PGS (** PGS    (dual) *)
  | MjSOL_CG (** CG     (primal) *)
  | MjSOL_NEWTON (** Newton (primal), *)

(** type of equality constraint *)
type mjtEq = Typs.mjtEq =
  | MjEQ_CONNECT (** connect two bodies at a point (ball joint) *)
  | MjEQ_WELD (** fix relative position and orientation of two bodies *)
  | MjEQ_JOINT (** couple the values of two scalar joints with cubic *)
  | MjEQ_TENDON (** couple the lengths of two tendons with cubic *)
  | MjEQ_DISTANCE (** fix the contact distance betweent two geoms, *)

(** type of tendon wrap object *)
type mjtWrap = Typs.mjtWrap =
  | MjWRAP_NONE (** null object *)
  | MjWRAP_JOINT (** constant moment arm *)
  | MjWRAP_PULLEY (** pulley used to split tendon *)
  | MjWRAP_SITE (** pass through site *)
  | MjWRAP_SPHERE (** wrap around sphere *)
  | MjWRAP_CYLINDER (** wrap around (infinite) cylinder, *)

(** type of actuator transmission *)
type mjtTrn = Typs.mjtTrn =
  | MjTRN_JOINT (** force on joint *)
  | MjTRN_JOINTINPARENT (** force on joint, expressed in parent frame *)
  | MjTRN_SLIDERCRANK (** force via slider-crank linkage *)
  | MjTRN_TENDON (** force on tendon *)
  | MjTRN_SITE (** force on site *)
  | MjTRN_UNDEFINED (** undefined transmission type, *)

(** type of actuator dynamics *)
type mjtDyn = Typs.mjtDyn =
  | MjDYN_NONE (** no internal dynamics; ctrl specifies force *)
  | MjDYN_INTEGRATOR (** integrator: da/dt = u *)
  | MjDYN_FILTER (** linear filter: da/dt = (u-a) / tau *)
  | MjDYN_MUSCLE (** piece-wise linear filter with two time constants *)
  | MjDYN_USER (** user-defined dynamics type, *)

(** type of actuator gain *)
type mjtGain = Typs.mjtGain =
  | MjGAIN_FIXED (** fixed gain *)
  | MjGAIN_MUSCLE (** muscle FLV curve computed by mju_muscleGain() *)
  | MjGAIN_USER (** user-defined gain type, *)

(** type of actuator bias *)
type mjtBias = Typs.mjtBias =
  | MjBIAS_NONE (** no bias *)
  | MjBIAS_AFFINE (** const + kp*length + kv*velocity *)
  | MjBIAS_MUSCLE (** muscle passive force computed by mju_muscleBias() *)
  | MjBIAS_USER (** user-defined bias type, *)

(** type of MujoCo object *)
type mjtObj = Typs.mjtObj =
  | MjOBJ_UNKNOWN (** unknown object type *)
  | MjOBJ_BODY (** body *)
  | MjOBJ_XBODY (** body, used to access regular frame instead of i-frame *)
  | MjOBJ_JOINT (** joint *)
  | MjOBJ_DOF (** dof *)
  | MjOBJ_GEOM (** geom *)
  | MjOBJ_SITE (** site *)
  | MjOBJ_CAMERA (** camera *)
  | MjOBJ_LIGHT (** light *)
  | MjOBJ_MESH (** mesh *)
  | MjOBJ_SKIN (** skin *)
  | MjOBJ_HFIELD (** heightfield *)
  | MjOBJ_TEXTURE (** texture *)
  | MjOBJ_MATERIAL (** material for rendering *)
  | MjOBJ_PAIR (** geom pair to include *)
  | MjOBJ_EXCLUDE (** body pair to exclude *)
  | MjOBJ_EQUALITY (** equality constraint *)
  | MjOBJ_TENDON (** tendon *)
  | MjOBJ_ACTUATOR (** actuator *)
  | MjOBJ_SENSOR (** sensor *)
  | MjOBJ_NUMERIC (** numeric *)
  | MjOBJ_TEXT (** text *)
  | MjOBJ_TUPLE (** tuple *)
  | MjOBJ_KEY (** keyframe, *)

(** type of constraint *)
type mjtConstraint = Typs.mjtConstraint =
  | MjCNSTR_EQUALITY (** equality constraint *)
  | MjCNSTR_FRICTION_DOF (** dof friction *)
  | MjCNSTR_FRICTION_TENDON (** tendon friction *)
  | MjCNSTR_LIMIT_JOINT (** joint limit *)
  | MjCNSTR_LIMIT_TENDON (** tendon limit *)
  | MjCNSTR_CONTACT_FRICTIONLESS (** frictionless contact *)
  | MjCNSTR_CONTACT_PYRAMIDAL (** frictional contact, pyramidal friction cone *)
  | MjCNSTR_CONTACT_ELLIPTIC (** frictional contact, elliptic friction cone, *)

(** constraint state *)
type mjtConstraintState = Typs.mjtConstraintState =
  | MjCNSTRSTATE_SATISFIED (** constraint satisfied, zero cost (limit, contact) *)
  | MjCNSTRSTATE_QUADRATIC (** quadratic cost (equality, friction, limit, contact) *)
  | MjCNSTRSTATE_LINEARNEG (** linear cost, negative side (friction) *)
  | MjCNSTRSTATE_LINEARPOS (** linear cost, positive side (friction) *)
  | MjCNSTRSTATE_CONE (** squared distance to cone cost (elliptic contact), *)

(** type of sensor *)
type mjtSensor = Typs.mjtSensor =
  | MjSENS_TOUCH (** scalar contact normal forces summed over sensor zone *)
  | MjSENS_ACCELEROMETER (** 3D linear acceleration, in local frame *)
  | MjSENS_VELOCIMETER (** 3D linear velocity, in local frame *)
  | MjSENS_GYRO (** 3D angular velocity, in local frame *)
  | MjSENS_FORCE (** 3D force between site's body and its parent body *)
  | MjSENS_TORQUE (** 3D torque between site's body and its parent body *)
  | MjSENS_MAGNETOMETER (** 3D magnetometer *)
  | MjSENS_RANGEFINDER (** scalar distance to nearest geom or site along z-axis *)
  | MjSENS_JOINTPOS (** scalar joint position (hinge and slide only) *)
  | MjSENS_JOINTVEL (** scalar joint velocity (hinge and slide only) *)
  | MjSENS_TENDONPOS (** scalar tendon position *)
  | MjSENS_TENDONVEL (** scalar tendon velocity *)
  | MjSENS_ACTUATORPOS (** scalar actuator position *)
  | MjSENS_ACTUATORVEL (** scalar actuator velocity *)
  | MjSENS_ACTUATORFRC (** scalar actuator force *)
  | MjSENS_BALLQUAT (** 4D ball joint quaterion *)
  | MjSENS_BALLANGVEL (** 3D ball joint angular velocity *)
  | MjSENS_JOINTLIMITPOS (** joint limit distance-margin *)
  | MjSENS_JOINTLIMITVEL (** joint limit velocity *)
  | MjSENS_JOINTLIMITFRC (** joint limit force *)
  | MjSENS_TENDONLIMITPOS (** tendon limit distance-margin *)
  | MjSENS_TENDONLIMITVEL (** tendon limit velocity *)
  | MjSENS_TENDONLIMITFRC (** tendon limit force *)
  | MjSENS_FRAMEPOS (** 3D position *)
  | MjSENS_FRAMEQUAT (** 4D unit quaternion orientation *)
  | MjSENS_FRAMEXAXIS (** 3D unit vector: x-axis of object's frame *)
  | MjSENS_FRAMEYAXIS (** 3D unit vector: y-axis of object's frame *)
  | MjSENS_FRAMEZAXIS (** 3D unit vector: z-axis of object's frame *)
  | MjSENS_FRAMELINVEL (** 3D linear velocity *)
  | MjSENS_FRAMEANGVEL (** 3D angular velocity *)
  | MjSENS_FRAMELINACC (** 3D linear acceleration *)
  | MjSENS_FRAMEANGACC (** 3D angular acceleration *)
  | MjSENS_SUBTREECOM (** 3D center of mass of subtree *)
  | MjSENS_SUBTREELINVEL (** 3D linear velocity of subtree *)
  | MjSENS_SUBTREEANGMOM (** 3D angular momentum of subtree *)
  | MjSENS_USER (** sensor data provided by mjcb_sensor callback, *)

(** computation stage *)
type mjtStage = Typs.mjtStage =
  | MjSTAGE_NONE (** no computations *)
  | MjSTAGE_POS (** position-dependent computations *)
  | MjSTAGE_VEL (** velocity-dependent computations *)
  | MjSTAGE_ACC (** acceleration/force-dependent computations, *)

(** data type for sensors *)
type mjtDataType = Typs.mjtDataType =
  | MjDATATYPE_REAL (** real values, no constraints *)
  | MjDATATYPE_POSITIVE (** positive values; 0 or negative: inactive *)
  | MjDATATYPE_AXIS (** 3D unit vector *)
  | MjDATATYPE_QUATERNION (** unit quaternion, *)

(** mode for actuator length range computation *)
type mjtLRMode = Typs.mjtLRMode =
  | MjLRMODE_NONE (** do not process any actuators *)
  | MjLRMODE_MUSCLE (** process muscle actuators *)
  | MjLRMODE_MUSCLEUSER (** process muscle and user actuators *)
  | MjLRMODE_ALL (** process all actuators, *)

(** type mjLROpt *)
type mjLROpt = Typs.mjLROpt t

(** type mjVFS *)
type mjVFS = Typs.mjVFS t

(** type mjOption *)
type mjOption = Typs.mjOption t

(** type mjVisual *)
type mjVisual = Typs.mjVisual t

(** type mjStatistic *)
type mjStatistic = Typs.mjStatistic t

(** type mjModel *)
type mjModel = Typs.mjModel t

(** warning types *)
type mjtWarning = Typs.mjtWarning =
  | MjWARN_INERTIA (** (near) singular inertia matrix *)
  | MjWARN_CONTACTFULL (** too many contacts in contact list *)
  | MjWARN_CNSTRFULL (** too many constraints *)
  | MjWARN_VGEOMFULL (** too many visual geoms *)
  | MjWARN_BADQPOS (** bad number in qpos *)
  | MjWARN_BADQVEL (** bad number in qvel *)
  | MjWARN_BADQACC (** bad number in qacc *)
  | MjWARN_BADCTRL (** bad number in ctrl *)
  | MjNWARNING (** number of warnings, *)

(** mjtTimer *)
type mjtTimer = Typs.mjtTimer =
  | MjTIMER_STEP (** step *)
  | MjTIMER_FORWARD (** forward *)
  | MjTIMER_INVERSE (** inverse *)
  | MjTIMER_POSITION (** fwdPosition *)
  | MjTIMER_VELOCITY (** fwdVelocity *)
  | MjTIMER_ACTUATION (** fwdActuation *)
  | MjTIMER_ACCELERATION (** fwdAcceleration *)
  | MjTIMER_CONSTRAINT (** fwdConstraint *)
  | MjTIMER_POS_KINEMATICS (** kinematics, com, tendon, transmission *)
  | MjTIMER_POS_INERTIA (** inertia computations *)
  | MjTIMER_POS_COLLISION (** collision detection *)
  | MjTIMER_POS_MAKE (** make constraints *)
  | MjTIMER_POS_PROJECT (** project constraints *)
  | MjNTIMER (** number of timers, *)

(** type mjContact *)
type mjContact = Typs.mjContact t

(** type mjWarningStat *)
type mjWarningStat = Typs.mjWarningStat t

(** type mjTimerStat *)
type mjTimerStat = Typs.mjTimerStat t

(** type mjSolverStat *)
type mjSolverStat = Typs.mjSolverStat t

(** type mjData *)
type mjData = Typs.mjData t

(** bitflags for mjvGeom category *)
type mjtCatBit = Typs.mjtCatBit =
  | MjCAT_STATIC (** model elements in body 0 *)
  | MjCAT_DYNAMIC (** model elements in all other bodies *)
  | MjCAT_DECOR (** decorative geoms *)
  | MjCAT_ALL (** select all categories, *)

(** mouse interaction mode *)
type mjtMouse = Typs.mjtMouse =
  | MjMOUSE_NONE (** no action *)
  | MjMOUSE_ROTATE_V (** rotate, vertical plane *)
  | MjMOUSE_ROTATE_H (** rotate, horizontal plane *)
  | MjMOUSE_MOVE_V (** move, vertical plane *)
  | MjMOUSE_MOVE_H (** move, horizontal plane *)
  | MjMOUSE_ZOOM (** zoom *)
  | MjMOUSE_SELECT (** selection, *)

(** mouse perturbations *)
type mjtPertBit = Typs.mjtPertBit =
  | MjPERT_TRANSLATE (** translation *)
  | MjPERT_ROTATE (** rotation, *)

(** abstract camera type *)
type mjtCamera = Typs.mjtCamera =
  | MjCAMERA_FREE (** free camera *)
  | MjCAMERA_TRACKING (** tracking camera; uses trackbodyid *)
  | MjCAMERA_FIXED (** fixed camera; uses fixedcamid *)
  | MjCAMERA_USER (** user is responsible for setting OpenGL camera, *)

(** object labeling *)
type mjtLabel = Typs.mjtLabel =
  | MjLABEL_NONE (** nothing *)
  | MjLABEL_BODY (** body labels *)
  | MjLABEL_JOINT (** joint labels *)
  | MjLABEL_GEOM (** geom labels *)
  | MjLABEL_SITE (** site labels *)
  | MjLABEL_CAMERA (** camera labels *)
  | MjLABEL_LIGHT (** light labels *)
  | MjLABEL_TENDON (** tendon labels *)
  | MjLABEL_ACTUATOR (** actuator labels *)
  | MjLABEL_CONSTRAINT (** constraint labels *)
  | MjLABEL_SKIN (** skin labels *)
  | MjLABEL_SELECTION (** selected object *)
  | MjLABEL_SELPNT (** coordinates of selection point *)
  | MjLABEL_CONTACTFORCE (** magnitude of contact force *)
  | MjNLABEL (** number of label types, *)

(** frame visualization *)
type mjtFrame = Typs.mjtFrame =
  | MjFRAME_NONE (** no frames *)
  | MjFRAME_BODY (** body frames *)
  | MjFRAME_GEOM (** geom frames *)
  | MjFRAME_SITE (** site frames *)
  | MjFRAME_CAMERA (** camera frames *)
  | MjFRAME_LIGHT (** light frames *)
  | MjFRAME_WORLD (** world frame *)
  | MjNFRAME (** number of visualization frames, *)

(** flags enabling model element visualization *)
type mjtVisFlag = Typs.mjtVisFlag =
  | MjVIS_CONVEXHULL (** mesh convex hull *)
  | MjVIS_TEXTURE (** textures *)
  | MjVIS_JOINT (** joints *)
  | MjVIS_ACTUATOR (** actuators *)
  | MjVIS_CAMERA (** cameras *)
  | MjVIS_LIGHT (** lights *)
  | MjVIS_TENDON (** tendons *)
  | MjVIS_RANGEFINDER (** rangefinder sensors *)
  | MjVIS_CONSTRAINT (** point constraints *)
  | MjVIS_INERTIA (** equivalent inertia boxes *)
  | MjVIS_SCLINERTIA (** scale equivalent inertia boxes with mass *)
  | MjVIS_PERTFORCE (** perturbation force *)
  | MjVIS_PERTOBJ (** perturbation object *)
  | MjVIS_CONTACTPOINT (** contact points *)
  | MjVIS_CONTACTFORCE (** contact force *)
  | MjVIS_CONTACTSPLIT (** split contact force into normal and tanget *)
  | MjVIS_TRANSPARENT (** make dynamic geoms more transparent *)
  | MjVIS_AUTOCONNECT (** auto connect joints and body coms *)
  | MjVIS_COM (** center of mass *)
  | MjVIS_SELECT (** selection point *)
  | MjVIS_STATIC (** static bodies *)
  | MjVIS_SKIN (** skin *)
  | MjNVISFLAG (** number of visualization flags, *)

(** flags enabling rendering effects *)
type mjtRndFlag = Typs.mjtRndFlag =
  | MjRND_SHADOW (** shadows *)
  | MjRND_WIREFRAME (** wireframe *)
  | MjRND_REFLECTION (** reflections *)
  | MjRND_ADDITIVE (** additive transparency *)
  | MjRND_SKYBOX (** skybox *)
  | MjRND_FOG (** fog *)
  | MjRND_HAZE (** haze *)
  | MjRND_SEGMENT (** segmentation with random color *)
  | MjRND_IDCOLOR (** segmentation with segid color *)
  | MjNRNDFLAG (** number of rendering flags, *)

(** type of stereo rendering *)
type mjtStereo = Typs.mjtStereo =
  | MjSTEREO_NONE (** no stereo; use left eye only *)
  | MjSTEREO_QUADBUFFERED
      (** quad buffered; revert to side-by-side if no hardware support *)
  | MjSTEREO_SIDEBYSIDE (** side-by-side, *)

(** type mjvPerturb *)
type mjvPerturb = Typs.mjvPerturb t

(** type mjvCamera *)
type mjvCamera = Typs.mjvCamera t

(** type mjvGLCamera *)
type mjvGLCamera = Typs.mjvGLCamera t

(** type mjvGeom *)
type mjvGeom = Typs.mjvGeom t

(** type mjvLight *)
type mjvLight = Typs.mjvLight t

(** type mjvOption *)
type mjvOption = Typs.mjvOption t

(** type mjvScene *)
type mjvScene = Typs.mjvScene t

(** type mjvFigure *)
type mjvFigure = Typs.mjvFigure t

(** grid position for overlay *)
type mjtGridPos = Typs.mjtGridPos =
  | MjGRID_TOPLEFT (** top left *)
  | MjGRID_TOPRIGHT (** top right *)
  | MjGRID_BOTTOMLEFT (** bottom left *)
  | MjGRID_BOTTOMRIGHT (** bottom right, *)

(** OpenGL framebuffer option *)
type mjtFramebuffer = Typs.mjtFramebuffer =
  | MjFB_WINDOW (** default/window buffer *)
  | MjFB_OFFSCREEN (** offscreen buffer, *)

(** font scale, used at context creation *)
type mjtFontScale = Typs.mjtFontScale =
  | MjFONTSCALE_50 (** 50% scale, suitable for low-res rendering *)
  | MjFONTSCALE_100 (** normal scale, suitable in the absence of DPI scaling *)
  | MjFONTSCALE_150 (** 150% scale *)
  | MjFONTSCALE_200 (** 200% scale *)
  | MjFONTSCALE_250 (** 250% scale *)
  | MjFONTSCALE_300 (** 300% scale, *)

(** font type, used at each text operation *)
type mjtFont = Typs.mjtFont =
  | MjFONT_NORMAL (** normal font *)
  | MjFONT_SHADOW (** normal font with shadow (for higher contrast) *)
  | MjFONT_BIG (** big font (for user alerts), *)

(** type mjrRect *)
type mjrRect = Typs.mjrRect t

(** type mjrContext *)
type mjrContext = Typs.mjrContext t

(** mouse button *)
type mjtButton = Typs.mjtButton =
  | MjBUTTON_NONE (** no button *)
  | MjBUTTON_LEFT (** left button *)
  | MjBUTTON_RIGHT (** right button *)
  | MjBUTTON_MIDDLE (** middle button, *)

(** mouse and keyboard event type *)
type mjtEvent = Typs.mjtEvent =
  | MjEVENT_NONE (** no event *)
  | MjEVENT_MOVE (** mouse move *)
  | MjEVENT_PRESS (** mouse button press *)
  | MjEVENT_RELEASE (** mouse button release *)
  | MjEVENT_SCROLL (** scroll *)
  | MjEVENT_KEY (** key press *)
  | MjEVENT_RESIZE (** resize, *)

(** UI item type *)
type mjtItem = Typs.mjtItem =
  | MjITEM_END (** end of definition list (not an item) *)
  | MjITEM_SECTION (** section (not an item) *)
  | MjITEM_SEPARATOR (** separator *)
  | MjITEM_STATIC (** static text *)
  | MjITEM_BUTTON (** button *)
  | MjITEM_CHECKINT (** check box, int value *)
  | MjITEM_CHECKBYTE (** check box, mjtByte value *)
  | MjITEM_RADIO (** radio group *)
  | MjITEM_RADIOLINE (** radio group, single line *)
  | MjITEM_SELECT (** selection box *)
  | MjITEM_SLIDERINT (** slider, int value *)
  | MjITEM_SLIDERNUM (** slider, mjtNum value *)
  | MjITEM_EDITINT (** editable array, int values *)
  | MjITEM_EDITNUM (** editable array, mjtNum values *)
  | MjITEM_EDITTXT (** editable text *)
  | MjNITEM (** number of item types, *)

(** type mjuiState *)
type mjuiState = Typs.mjuiState t

(** type mjuiThemeSpacing *)
type mjuiThemeSpacing = Typs.mjuiThemeSpacing t

(** type mjuiThemeColor *)
type mjuiThemeColor = Typs.mjuiThemeColor t

(** type mjuiItemSingle *)
type mjuiItemSingle = Typs.mjuiItemSingle t

(** type mjuiItemMulti *)
type mjuiItemMulti = Typs.mjuiItemMulti t

(** type mjuiItemSlider *)
type mjuiItemSlider = Typs.mjuiItemSlider t

(** type mjuiItemEdit *)
type mjuiItemEdit = Typs.mjuiItemEdit t

(** type mjuiItem *)
type mjuiItem = Typs.mjuiItem t

(** type mjuiSection *)
type mjuiSection = Typs.mjuiSection t

(** type mjUI *)
type mjUI = Typs.mjUI t

(** type mjuiDef *)
type mjuiDef = Typs.mjuiDef t

(** convert mjtDisableBit type to int *)
let mjtDisableBit_to_int = function
  | MjDSBL_CONSTRAINT   -> Typs.mjDSBL_CONSTRAINT |> Int64.to_int
  | MjDSBL_EQUALITY     -> Typs.mjDSBL_EQUALITY |> Int64.to_int
  | MjDSBL_FRICTIONLOSS -> Typs.mjDSBL_FRICTIONLOSS |> Int64.to_int
  | MjDSBL_LIMIT        -> Typs.mjDSBL_LIMIT |> Int64.to_int
  | MjDSBL_CONTACT      -> Typs.mjDSBL_CONTACT |> Int64.to_int
  | MjDSBL_PASSIVE      -> Typs.mjDSBL_PASSIVE |> Int64.to_int
  | MjDSBL_GRAVITY      -> Typs.mjDSBL_GRAVITY |> Int64.to_int
  | MjDSBL_CLAMPCTRL    -> Typs.mjDSBL_CLAMPCTRL |> Int64.to_int
  | MjDSBL_WARMSTART    -> Typs.mjDSBL_WARMSTART |> Int64.to_int
  | MjDSBL_FILTERPARENT -> Typs.mjDSBL_FILTERPARENT |> Int64.to_int
  | MjDSBL_ACTUATION    -> Typs.mjDSBL_ACTUATION |> Int64.to_int
  | MjDSBL_REFSAFE      -> Typs.mjDSBL_REFSAFE |> Int64.to_int
  | MjNDISABLE          -> Typs.mjNDISABLE |> Int64.to_int


(** convert mjtEnableBit type to int *)
let mjtEnableBit_to_int = function
  | MjENBL_OVERRIDE    -> Typs.mjENBL_OVERRIDE |> Int64.to_int
  | MjENBL_ENERGY      -> Typs.mjENBL_ENERGY |> Int64.to_int
  | MjENBL_FWDINV      -> Typs.mjENBL_FWDINV |> Int64.to_int
  | MjENBL_SENSORNOISE -> Typs.mjENBL_SENSORNOISE |> Int64.to_int
  | MjNENABLE          -> Typs.mjNENABLE |> Int64.to_int


(** convert mjtJoint type to int *)
let mjtJoint_to_int = function
  | MjJNT_FREE  -> Typs.mjJNT_FREE |> Int64.to_int
  | MjJNT_BALL  -> Typs.mjJNT_BALL |> Int64.to_int
  | MjJNT_SLIDE -> Typs.mjJNT_SLIDE |> Int64.to_int
  | MjJNT_HINGE -> Typs.mjJNT_HINGE |> Int64.to_int


(** convert mjtGeom type to int *)
let mjtGeom_to_int = function
  | MjGEOM_PLANE     -> Typs.mjGEOM_PLANE |> Int64.to_int
  | MjGEOM_HFIELD    -> Typs.mjGEOM_HFIELD |> Int64.to_int
  | MjGEOM_SPHERE    -> Typs.mjGEOM_SPHERE |> Int64.to_int
  | MjGEOM_CAPSULE   -> Typs.mjGEOM_CAPSULE |> Int64.to_int
  | MjGEOM_ELLIPSOID -> Typs.mjGEOM_ELLIPSOID |> Int64.to_int
  | MjGEOM_CYLINDER  -> Typs.mjGEOM_CYLINDER |> Int64.to_int
  | MjGEOM_BOX       -> Typs.mjGEOM_BOX |> Int64.to_int
  | MjGEOM_MESH      -> Typs.mjGEOM_MESH |> Int64.to_int
  | MjNGEOMTYPES     -> Typs.mjNGEOMTYPES |> Int64.to_int
  | MjGEOM_ARROW     -> Typs.mjGEOM_ARROW |> Int64.to_int
  | MjGEOM_ARROW1    -> Typs.mjGEOM_ARROW1 |> Int64.to_int
  | MjGEOM_ARROW2    -> Typs.mjGEOM_ARROW2 |> Int64.to_int
  | MjGEOM_LINE      -> Typs.mjGEOM_LINE |> Int64.to_int
  | MjGEOM_SKIN      -> Typs.mjGEOM_SKIN |> Int64.to_int
  | MjGEOM_LABEL     -> Typs.mjGEOM_LABEL |> Int64.to_int
  | MjGEOM_NONE      -> Typs.mjGEOM_NONE |> Int64.to_int


(** convert mjtCamLight type to int *)
let mjtCamLight_to_int = function
  | MjCAMLIGHT_FIXED         -> Typs.mjCAMLIGHT_FIXED |> Int64.to_int
  | MjCAMLIGHT_TRACK         -> Typs.mjCAMLIGHT_TRACK |> Int64.to_int
  | MjCAMLIGHT_TRACKCOM      -> Typs.mjCAMLIGHT_TRACKCOM |> Int64.to_int
  | MjCAMLIGHT_TARGETBODY    -> Typs.mjCAMLIGHT_TARGETBODY |> Int64.to_int
  | MjCAMLIGHT_TARGETBODYCOM -> Typs.mjCAMLIGHT_TARGETBODYCOM |> Int64.to_int


(** convert mjtTexture type to int *)
let mjtTexture_to_int = function
  | MjTEXTURE_2D     -> Typs.mjTEXTURE_2D |> Int64.to_int
  | MjTEXTURE_CUBE   -> Typs.mjTEXTURE_CUBE |> Int64.to_int
  | MjTEXTURE_SKYBOX -> Typs.mjTEXTURE_SKYBOX |> Int64.to_int


(** convert mjtIntegrator type to int *)
let mjtIntegrator_to_int = function
  | MjINT_EULER -> Typs.mjINT_EULER |> Int64.to_int
  | MjINT_RK4   -> Typs.mjINT_RK4 |> Int64.to_int


(** convert mjtCollision type to int *)
let mjtCollision_to_int = function
  | MjCOL_ALL     -> Typs.mjCOL_ALL |> Int64.to_int
  | MjCOL_PAIR    -> Typs.mjCOL_PAIR |> Int64.to_int
  | MjCOL_DYNAMIC -> Typs.mjCOL_DYNAMIC |> Int64.to_int


(** convert mjtCone type to int *)
let mjtCone_to_int = function
  | MjCONE_PYRAMIDAL -> Typs.mjCONE_PYRAMIDAL |> Int64.to_int
  | MjCONE_ELLIPTIC  -> Typs.mjCONE_ELLIPTIC |> Int64.to_int


(** convert mjtJacobian type to int *)
let mjtJacobian_to_int = function
  | MjJAC_DENSE  -> Typs.mjJAC_DENSE |> Int64.to_int
  | MjJAC_SPARSE -> Typs.mjJAC_SPARSE |> Int64.to_int
  | MjJAC_AUTO   -> Typs.mjJAC_AUTO |> Int64.to_int


(** convert mjtSolver type to int *)
let mjtSolver_to_int = function
  | MjSOL_PGS    -> Typs.mjSOL_PGS |> Int64.to_int
  | MjSOL_CG     -> Typs.mjSOL_CG |> Int64.to_int
  | MjSOL_NEWTON -> Typs.mjSOL_NEWTON |> Int64.to_int


(** convert mjtEq type to int *)
let mjtEq_to_int = function
  | MjEQ_CONNECT  -> Typs.mjEQ_CONNECT |> Int64.to_int
  | MjEQ_WELD     -> Typs.mjEQ_WELD |> Int64.to_int
  | MjEQ_JOINT    -> Typs.mjEQ_JOINT |> Int64.to_int
  | MjEQ_TENDON   -> Typs.mjEQ_TENDON |> Int64.to_int
  | MjEQ_DISTANCE -> Typs.mjEQ_DISTANCE |> Int64.to_int


(** convert mjtWrap type to int *)
let mjtWrap_to_int = function
  | MjWRAP_NONE     -> Typs.mjWRAP_NONE |> Int64.to_int
  | MjWRAP_JOINT    -> Typs.mjWRAP_JOINT |> Int64.to_int
  | MjWRAP_PULLEY   -> Typs.mjWRAP_PULLEY |> Int64.to_int
  | MjWRAP_SITE     -> Typs.mjWRAP_SITE |> Int64.to_int
  | MjWRAP_SPHERE   -> Typs.mjWRAP_SPHERE |> Int64.to_int
  | MjWRAP_CYLINDER -> Typs.mjWRAP_CYLINDER |> Int64.to_int


(** convert mjtTrn type to int *)
let mjtTrn_to_int = function
  | MjTRN_JOINT         -> Typs.mjTRN_JOINT |> Int64.to_int
  | MjTRN_JOINTINPARENT -> Typs.mjTRN_JOINTINPARENT |> Int64.to_int
  | MjTRN_SLIDERCRANK   -> Typs.mjTRN_SLIDERCRANK |> Int64.to_int
  | MjTRN_TENDON        -> Typs.mjTRN_TENDON |> Int64.to_int
  | MjTRN_SITE          -> Typs.mjTRN_SITE |> Int64.to_int
  | MjTRN_UNDEFINED     -> Typs.mjTRN_UNDEFINED |> Int64.to_int


(** convert mjtDyn type to int *)
let mjtDyn_to_int = function
  | MjDYN_NONE       -> Typs.mjDYN_NONE |> Int64.to_int
  | MjDYN_INTEGRATOR -> Typs.mjDYN_INTEGRATOR |> Int64.to_int
  | MjDYN_FILTER     -> Typs.mjDYN_FILTER |> Int64.to_int
  | MjDYN_MUSCLE     -> Typs.mjDYN_MUSCLE |> Int64.to_int
  | MjDYN_USER       -> Typs.mjDYN_USER |> Int64.to_int


(** convert mjtGain type to int *)
let mjtGain_to_int = function
  | MjGAIN_FIXED  -> Typs.mjGAIN_FIXED |> Int64.to_int
  | MjGAIN_MUSCLE -> Typs.mjGAIN_MUSCLE |> Int64.to_int
  | MjGAIN_USER   -> Typs.mjGAIN_USER |> Int64.to_int


(** convert mjtBias type to int *)
let mjtBias_to_int = function
  | MjBIAS_NONE   -> Typs.mjBIAS_NONE |> Int64.to_int
  | MjBIAS_AFFINE -> Typs.mjBIAS_AFFINE |> Int64.to_int
  | MjBIAS_MUSCLE -> Typs.mjBIAS_MUSCLE |> Int64.to_int
  | MjBIAS_USER   -> Typs.mjBIAS_USER |> Int64.to_int


(** convert mjtObj type to int *)
let mjtObj_to_int = function
  | MjOBJ_UNKNOWN  -> Typs.mjOBJ_UNKNOWN |> Int64.to_int
  | MjOBJ_BODY     -> Typs.mjOBJ_BODY |> Int64.to_int
  | MjOBJ_XBODY    -> Typs.mjOBJ_XBODY |> Int64.to_int
  | MjOBJ_JOINT    -> Typs.mjOBJ_JOINT |> Int64.to_int
  | MjOBJ_DOF      -> Typs.mjOBJ_DOF |> Int64.to_int
  | MjOBJ_GEOM     -> Typs.mjOBJ_GEOM |> Int64.to_int
  | MjOBJ_SITE     -> Typs.mjOBJ_SITE |> Int64.to_int
  | MjOBJ_CAMERA   -> Typs.mjOBJ_CAMERA |> Int64.to_int
  | MjOBJ_LIGHT    -> Typs.mjOBJ_LIGHT |> Int64.to_int
  | MjOBJ_MESH     -> Typs.mjOBJ_MESH |> Int64.to_int
  | MjOBJ_SKIN     -> Typs.mjOBJ_SKIN |> Int64.to_int
  | MjOBJ_HFIELD   -> Typs.mjOBJ_HFIELD |> Int64.to_int
  | MjOBJ_TEXTURE  -> Typs.mjOBJ_TEXTURE |> Int64.to_int
  | MjOBJ_MATERIAL -> Typs.mjOBJ_MATERIAL |> Int64.to_int
  | MjOBJ_PAIR     -> Typs.mjOBJ_PAIR |> Int64.to_int
  | MjOBJ_EXCLUDE  -> Typs.mjOBJ_EXCLUDE |> Int64.to_int
  | MjOBJ_EQUALITY -> Typs.mjOBJ_EQUALITY |> Int64.to_int
  | MjOBJ_TENDON   -> Typs.mjOBJ_TENDON |> Int64.to_int
  | MjOBJ_ACTUATOR -> Typs.mjOBJ_ACTUATOR |> Int64.to_int
  | MjOBJ_SENSOR   -> Typs.mjOBJ_SENSOR |> Int64.to_int
  | MjOBJ_NUMERIC  -> Typs.mjOBJ_NUMERIC |> Int64.to_int
  | MjOBJ_TEXT     -> Typs.mjOBJ_TEXT |> Int64.to_int
  | MjOBJ_TUPLE    -> Typs.mjOBJ_TUPLE |> Int64.to_int
  | MjOBJ_KEY      -> Typs.mjOBJ_KEY |> Int64.to_int


(** convert mjtConstraint type to int *)
let mjtConstraint_to_int = function
  | MjCNSTR_EQUALITY             -> Typs.mjCNSTR_EQUALITY |> Int64.to_int
  | MjCNSTR_FRICTION_DOF         -> Typs.mjCNSTR_FRICTION_DOF |> Int64.to_int
  | MjCNSTR_FRICTION_TENDON      -> Typs.mjCNSTR_FRICTION_TENDON |> Int64.to_int
  | MjCNSTR_LIMIT_JOINT          -> Typs.mjCNSTR_LIMIT_JOINT |> Int64.to_int
  | MjCNSTR_LIMIT_TENDON         -> Typs.mjCNSTR_LIMIT_TENDON |> Int64.to_int
  | MjCNSTR_CONTACT_FRICTIONLESS -> Typs.mjCNSTR_CONTACT_FRICTIONLESS |> Int64.to_int
  | MjCNSTR_CONTACT_PYRAMIDAL    -> Typs.mjCNSTR_CONTACT_PYRAMIDAL |> Int64.to_int
  | MjCNSTR_CONTACT_ELLIPTIC     -> Typs.mjCNSTR_CONTACT_ELLIPTIC |> Int64.to_int


(** convert mjtConstraintState type to int *)
let mjtConstraintState_to_int = function
  | MjCNSTRSTATE_SATISFIED -> Typs.mjCNSTRSTATE_SATISFIED |> Int64.to_int
  | MjCNSTRSTATE_QUADRATIC -> Typs.mjCNSTRSTATE_QUADRATIC |> Int64.to_int
  | MjCNSTRSTATE_LINEARNEG -> Typs.mjCNSTRSTATE_LINEARNEG |> Int64.to_int
  | MjCNSTRSTATE_LINEARPOS -> Typs.mjCNSTRSTATE_LINEARPOS |> Int64.to_int
  | MjCNSTRSTATE_CONE      -> Typs.mjCNSTRSTATE_CONE |> Int64.to_int


(** convert mjtSensor type to int *)
let mjtSensor_to_int = function
  | MjSENS_TOUCH          -> Typs.mjSENS_TOUCH |> Int64.to_int
  | MjSENS_ACCELEROMETER  -> Typs.mjSENS_ACCELEROMETER |> Int64.to_int
  | MjSENS_VELOCIMETER    -> Typs.mjSENS_VELOCIMETER |> Int64.to_int
  | MjSENS_GYRO           -> Typs.mjSENS_GYRO |> Int64.to_int
  | MjSENS_FORCE          -> Typs.mjSENS_FORCE |> Int64.to_int
  | MjSENS_TORQUE         -> Typs.mjSENS_TORQUE |> Int64.to_int
  | MjSENS_MAGNETOMETER   -> Typs.mjSENS_MAGNETOMETER |> Int64.to_int
  | MjSENS_RANGEFINDER    -> Typs.mjSENS_RANGEFINDER |> Int64.to_int
  | MjSENS_JOINTPOS       -> Typs.mjSENS_JOINTPOS |> Int64.to_int
  | MjSENS_JOINTVEL       -> Typs.mjSENS_JOINTVEL |> Int64.to_int
  | MjSENS_TENDONPOS      -> Typs.mjSENS_TENDONPOS |> Int64.to_int
  | MjSENS_TENDONVEL      -> Typs.mjSENS_TENDONVEL |> Int64.to_int
  | MjSENS_ACTUATORPOS    -> Typs.mjSENS_ACTUATORPOS |> Int64.to_int
  | MjSENS_ACTUATORVEL    -> Typs.mjSENS_ACTUATORVEL |> Int64.to_int
  | MjSENS_ACTUATORFRC    -> Typs.mjSENS_ACTUATORFRC |> Int64.to_int
  | MjSENS_BALLQUAT       -> Typs.mjSENS_BALLQUAT |> Int64.to_int
  | MjSENS_BALLANGVEL     -> Typs.mjSENS_BALLANGVEL |> Int64.to_int
  | MjSENS_JOINTLIMITPOS  -> Typs.mjSENS_JOINTLIMITPOS |> Int64.to_int
  | MjSENS_JOINTLIMITVEL  -> Typs.mjSENS_JOINTLIMITVEL |> Int64.to_int
  | MjSENS_JOINTLIMITFRC  -> Typs.mjSENS_JOINTLIMITFRC |> Int64.to_int
  | MjSENS_TENDONLIMITPOS -> Typs.mjSENS_TENDONLIMITPOS |> Int64.to_int
  | MjSENS_TENDONLIMITVEL -> Typs.mjSENS_TENDONLIMITVEL |> Int64.to_int
  | MjSENS_TENDONLIMITFRC -> Typs.mjSENS_TENDONLIMITFRC |> Int64.to_int
  | MjSENS_FRAMEPOS       -> Typs.mjSENS_FRAMEPOS |> Int64.to_int
  | MjSENS_FRAMEQUAT      -> Typs.mjSENS_FRAMEQUAT |> Int64.to_int
  | MjSENS_FRAMEXAXIS     -> Typs.mjSENS_FRAMEXAXIS |> Int64.to_int
  | MjSENS_FRAMEYAXIS     -> Typs.mjSENS_FRAMEYAXIS |> Int64.to_int
  | MjSENS_FRAMEZAXIS     -> Typs.mjSENS_FRAMEZAXIS |> Int64.to_int
  | MjSENS_FRAMELINVEL    -> Typs.mjSENS_FRAMELINVEL |> Int64.to_int
  | MjSENS_FRAMEANGVEL    -> Typs.mjSENS_FRAMEANGVEL |> Int64.to_int
  | MjSENS_FRAMELINACC    -> Typs.mjSENS_FRAMELINACC |> Int64.to_int
  | MjSENS_FRAMEANGACC    -> Typs.mjSENS_FRAMEANGACC |> Int64.to_int
  | MjSENS_SUBTREECOM     -> Typs.mjSENS_SUBTREECOM |> Int64.to_int
  | MjSENS_SUBTREELINVEL  -> Typs.mjSENS_SUBTREELINVEL |> Int64.to_int
  | MjSENS_SUBTREEANGMOM  -> Typs.mjSENS_SUBTREEANGMOM |> Int64.to_int
  | MjSENS_USER           -> Typs.mjSENS_USER |> Int64.to_int


(** convert mjtStage type to int *)
let mjtStage_to_int = function
  | MjSTAGE_NONE -> Typs.mjSTAGE_NONE |> Int64.to_int
  | MjSTAGE_POS  -> Typs.mjSTAGE_POS |> Int64.to_int
  | MjSTAGE_VEL  -> Typs.mjSTAGE_VEL |> Int64.to_int
  | MjSTAGE_ACC  -> Typs.mjSTAGE_ACC |> Int64.to_int


(** convert mjtDataType type to int *)
let mjtDataType_to_int = function
  | MjDATATYPE_REAL       -> Typs.mjDATATYPE_REAL |> Int64.to_int
  | MjDATATYPE_POSITIVE   -> Typs.mjDATATYPE_POSITIVE |> Int64.to_int
  | MjDATATYPE_AXIS       -> Typs.mjDATATYPE_AXIS |> Int64.to_int
  | MjDATATYPE_QUATERNION -> Typs.mjDATATYPE_QUATERNION |> Int64.to_int


(** convert mjtLRMode type to int *)
let mjtLRMode_to_int = function
  | MjLRMODE_NONE       -> Typs.mjLRMODE_NONE |> Int64.to_int
  | MjLRMODE_MUSCLE     -> Typs.mjLRMODE_MUSCLE |> Int64.to_int
  | MjLRMODE_MUSCLEUSER -> Typs.mjLRMODE_MUSCLEUSER |> Int64.to_int
  | MjLRMODE_ALL        -> Typs.mjLRMODE_ALL |> Int64.to_int


(** allocate fresh mjLROpt struct *)
let mjLROpt_allocate () = Ctypes.(make Typs.mjLROpt)

(** make null mjLROpt struct ptr *)
let mjLROpt_null () = Ctypes.(from_voidp Typs.mjLROpt null)

(** get mode from mjLROpt *)
let mjLROpt_get_mode x = Ctypes.(getf x Typs.mjLROpt_mode)

(** set mode for mjLROpt *)
let mjLROpt_set_mode x y = Ctypes.(setf x Typs.mjLROpt_mode y)

(** get useexisting from mjLROpt *)
let mjLROpt_get_useexisting x = Ctypes.(getf x Typs.mjLROpt_useexisting)

(** set useexisting for mjLROpt *)
let mjLROpt_set_useexisting x y = Ctypes.(setf x Typs.mjLROpt_useexisting y)

(** get uselimit from mjLROpt *)
let mjLROpt_get_uselimit x = Ctypes.(getf x Typs.mjLROpt_uselimit)

(** set uselimit for mjLROpt *)
let mjLROpt_set_uselimit x y = Ctypes.(setf x Typs.mjLROpt_uselimit y)

(** get accel from mjLROpt *)
let mjLROpt_get_accel x = Ctypes.(getf x Typs.mjLROpt_accel)

(** set accel for mjLROpt *)
let mjLROpt_set_accel x y = Ctypes.(setf x Typs.mjLROpt_accel y)

(** get maxforce from mjLROpt *)
let mjLROpt_get_maxforce x = Ctypes.(getf x Typs.mjLROpt_maxforce)

(** set maxforce for mjLROpt *)
let mjLROpt_set_maxforce x y = Ctypes.(setf x Typs.mjLROpt_maxforce y)

(** get timeconst from mjLROpt *)
let mjLROpt_get_timeconst x = Ctypes.(getf x Typs.mjLROpt_timeconst)

(** set timeconst for mjLROpt *)
let mjLROpt_set_timeconst x y = Ctypes.(setf x Typs.mjLROpt_timeconst y)

(** get timestep from mjLROpt *)
let mjLROpt_get_timestep x = Ctypes.(getf x Typs.mjLROpt_timestep)

(** set timestep for mjLROpt *)
let mjLROpt_set_timestep x y = Ctypes.(setf x Typs.mjLROpt_timestep y)

(** get inttotal from mjLROpt *)
let mjLROpt_get_inttotal x = Ctypes.(getf x Typs.mjLROpt_inttotal)

(** set inttotal for mjLROpt *)
let mjLROpt_set_inttotal x y = Ctypes.(setf x Typs.mjLROpt_inttotal y)

(** get inteval from mjLROpt *)
let mjLROpt_get_inteval x = Ctypes.(getf x Typs.mjLROpt_inteval)

(** set inteval for mjLROpt *)
let mjLROpt_set_inteval x y = Ctypes.(setf x Typs.mjLROpt_inteval y)

(** get tolrange from mjLROpt *)
let mjLROpt_get_tolrange x = Ctypes.(getf x Typs.mjLROpt_tolrange)

(** set tolrange for mjLROpt *)
let mjLROpt_set_tolrange x y = Ctypes.(setf x Typs.mjLROpt_tolrange y)

(** make mjLROpt struct *)
let mjLROpt_make
    ?mjf_mode
    ?mjf_useexisting
    ?mjf_uselimit
    ?mjf_accel
    ?mjf_maxforce
    ?mjf_timeconst
    ?mjf_timestep
    ?mjf_inttotal
    ?mjf_inteval
    ?mjf_tolrange
    ()
  =
  let x = mjLROpt_allocate () in
  Option.iter (mjLROpt_set_mode x) mjf_mode;
  Option.iter (mjLROpt_set_useexisting x) mjf_useexisting;
  Option.iter (mjLROpt_set_uselimit x) mjf_uselimit;
  Option.iter (mjLROpt_set_accel x) mjf_accel;
  Option.iter (mjLROpt_set_maxforce x) mjf_maxforce;
  Option.iter (mjLROpt_set_timeconst x) mjf_timeconst;
  Option.iter (mjLROpt_set_timestep x) mjf_timestep;
  Option.iter (mjLROpt_set_inttotal x) mjf_inttotal;
  Option.iter (mjLROpt_set_inteval x) mjf_inteval;
  Option.iter (mjLROpt_set_tolrange x) mjf_tolrange;
  x


(** returns mjLROpt carray from list of mjLROpt *)
let mjLROpt_to_carray = Ctypes.CArray.of_list Typs.mjLROpt

(** allocate fresh mjVFS struct *)
let mjVFS_allocate () = Ctypes.(make Typs.mjVFS)

(** make null mjVFS struct ptr *)
let mjVFS_null () = Ctypes.(from_voidp Typs.mjVFS null)

(** get nfile from mjVFS *)
let mjVFS_get_nfile x = Ctypes.(getf x Typs.mjVFS_nfile)

(** set nfile for mjVFS *)
let mjVFS_set_nfile x y = Ctypes.(setf x Typs.mjVFS_nfile y)

(** get filesize from mjVFS *)
let mjVFS_get_filesize x = Ctypes.(getf x Typs.mjVFS_filesize)

(** set filesize for mjVFS *)
let mjVFS_set_filesize x y = Ctypes.(setf x Typs.mjVFS_filesize y)

(** get filedata from mjVFS *)
let mjVFS_get_filedata x = Ctypes.(getf x Typs.mjVFS_filedata)

(** set filedata for mjVFS *)
let mjVFS_set_filedata x y = Ctypes.(setf x Typs.mjVFS_filedata y)

(** make mjVFS struct *)
let mjVFS_make ?mjf_nfile ?mjf_filesize ?mjf_filedata () =
  let x = mjVFS_allocate () in
  Option.iter (mjVFS_set_nfile x) mjf_nfile;
  Option.iter (mjVFS_set_filesize x) mjf_filesize;
  Option.iter (mjVFS_set_filedata x) mjf_filedata;
  x


(** returns mjVFS carray from list of mjVFS *)
let mjVFS_to_carray = Ctypes.CArray.of_list Typs.mjVFS

(** allocate fresh mjOption struct *)
let mjOption_allocate () = Ctypes.(make Typs.mjOption)

(** make null mjOption struct ptr *)
let mjOption_null () = Ctypes.(from_voidp Typs.mjOption null)

(** get timestep from mjOption *)
let mjOption_get_timestep x = Ctypes.(getf x Typs.mjOption_timestep)

(** set timestep for mjOption *)
let mjOption_set_timestep x y = Ctypes.(setf x Typs.mjOption_timestep y)

(** get apirate from mjOption *)
let mjOption_get_apirate x = Ctypes.(getf x Typs.mjOption_apirate)

(** set apirate for mjOption *)
let mjOption_set_apirate x y = Ctypes.(setf x Typs.mjOption_apirate y)

(** get impratio from mjOption *)
let mjOption_get_impratio x = Ctypes.(getf x Typs.mjOption_impratio)

(** set impratio for mjOption *)
let mjOption_set_impratio x y = Ctypes.(setf x Typs.mjOption_impratio y)

(** get tolerance from mjOption *)
let mjOption_get_tolerance x = Ctypes.(getf x Typs.mjOption_tolerance)

(** set tolerance for mjOption *)
let mjOption_set_tolerance x y = Ctypes.(setf x Typs.mjOption_tolerance y)

(** get noslip_tolerance from mjOption *)
let mjOption_get_noslip_tolerance x = Ctypes.(getf x Typs.mjOption_noslip_tolerance)

(** set noslip_tolerance for mjOption *)
let mjOption_set_noslip_tolerance x y = Ctypes.(setf x Typs.mjOption_noslip_tolerance y)

(** get mpr_tolerance from mjOption *)
let mjOption_get_mpr_tolerance x = Ctypes.(getf x Typs.mjOption_mpr_tolerance)

(** set mpr_tolerance for mjOption *)
let mjOption_set_mpr_tolerance x y = Ctypes.(setf x Typs.mjOption_mpr_tolerance y)

(** get gravity from mjOption *)
let mjOption_get_gravity x = Ctypes.(getf x Typs.mjOption_gravity)

(** set gravity for mjOption *)
let mjOption_set_gravity x y = Ctypes.(setf x Typs.mjOption_gravity y)

(** get wind from mjOption *)
let mjOption_get_wind x = Ctypes.(getf x Typs.mjOption_wind)

(** set wind for mjOption *)
let mjOption_set_wind x y = Ctypes.(setf x Typs.mjOption_wind y)

(** get magnetic from mjOption *)
let mjOption_get_magnetic x = Ctypes.(getf x Typs.mjOption_magnetic)

(** set magnetic for mjOption *)
let mjOption_set_magnetic x y = Ctypes.(setf x Typs.mjOption_magnetic y)

(** get density from mjOption *)
let mjOption_get_density x = Ctypes.(getf x Typs.mjOption_density)

(** set density for mjOption *)
let mjOption_set_density x y = Ctypes.(setf x Typs.mjOption_density y)

(** get viscosity from mjOption *)
let mjOption_get_viscosity x = Ctypes.(getf x Typs.mjOption_viscosity)

(** set viscosity for mjOption *)
let mjOption_set_viscosity x y = Ctypes.(setf x Typs.mjOption_viscosity y)

(** get o_margin from mjOption *)
let mjOption_get_o_margin x = Ctypes.(getf x Typs.mjOption_o_margin)

(** set o_margin for mjOption *)
let mjOption_set_o_margin x y = Ctypes.(setf x Typs.mjOption_o_margin y)

(** get o_solref from mjOption *)
let mjOption_get_o_solref x = Ctypes.(getf x Typs.mjOption_o_solref)

(** set o_solref for mjOption *)
let mjOption_set_o_solref x y = Ctypes.(setf x Typs.mjOption_o_solref y)

(** get o_solimp from mjOption *)
let mjOption_get_o_solimp x = Ctypes.(getf x Typs.mjOption_o_solimp)

(** set o_solimp for mjOption *)
let mjOption_set_o_solimp x y = Ctypes.(setf x Typs.mjOption_o_solimp y)

(** get integrator from mjOption *)
let mjOption_get_integrator x = Ctypes.(getf x Typs.mjOption_integrator)

(** set integrator for mjOption *)
let mjOption_set_integrator x y = Ctypes.(setf x Typs.mjOption_integrator y)

(** get collision from mjOption *)
let mjOption_get_collision x = Ctypes.(getf x Typs.mjOption_collision)

(** set collision for mjOption *)
let mjOption_set_collision x y = Ctypes.(setf x Typs.mjOption_collision y)

(** get cone from mjOption *)
let mjOption_get_cone x = Ctypes.(getf x Typs.mjOption_cone)

(** set cone for mjOption *)
let mjOption_set_cone x y = Ctypes.(setf x Typs.mjOption_cone y)

(** get jacobian from mjOption *)
let mjOption_get_jacobian x = Ctypes.(getf x Typs.mjOption_jacobian)

(** set jacobian for mjOption *)
let mjOption_set_jacobian x y = Ctypes.(setf x Typs.mjOption_jacobian y)

(** get solver from mjOption *)
let mjOption_get_solver x = Ctypes.(getf x Typs.mjOption_solver)

(** set solver for mjOption *)
let mjOption_set_solver x y = Ctypes.(setf x Typs.mjOption_solver y)

(** get iterations from mjOption *)
let mjOption_get_iterations x = Ctypes.(getf x Typs.mjOption_iterations)

(** set iterations for mjOption *)
let mjOption_set_iterations x y = Ctypes.(setf x Typs.mjOption_iterations y)

(** get noslip_iterations from mjOption *)
let mjOption_get_noslip_iterations x = Ctypes.(getf x Typs.mjOption_noslip_iterations)

(** set noslip_iterations for mjOption *)
let mjOption_set_noslip_iterations x y = Ctypes.(setf x Typs.mjOption_noslip_iterations y)

(** get mpr_iterations from mjOption *)
let mjOption_get_mpr_iterations x = Ctypes.(getf x Typs.mjOption_mpr_iterations)

(** set mpr_iterations for mjOption *)
let mjOption_set_mpr_iterations x y = Ctypes.(setf x Typs.mjOption_mpr_iterations y)

(** get disableflags from mjOption *)
let mjOption_get_disableflags x = Ctypes.(getf x Typs.mjOption_disableflags)

(** set disableflags for mjOption *)
let mjOption_set_disableflags x y = Ctypes.(setf x Typs.mjOption_disableflags y)

(** get enableflags from mjOption *)
let mjOption_get_enableflags x = Ctypes.(getf x Typs.mjOption_enableflags)

(** set enableflags for mjOption *)
let mjOption_set_enableflags x y = Ctypes.(setf x Typs.mjOption_enableflags y)

(** make mjOption struct *)
let mjOption_make
    ?mjf_timestep
    ?mjf_apirate
    ?mjf_impratio
    ?mjf_tolerance
    ?mjf_noslip_tolerance
    ?mjf_mpr_tolerance
    ?mjf_gravity
    ?mjf_wind
    ?mjf_magnetic
    ?mjf_density
    ?mjf_viscosity
    ?mjf_o_margin
    ?mjf_o_solref
    ?mjf_o_solimp
    ?mjf_integrator
    ?mjf_collision
    ?mjf_cone
    ?mjf_jacobian
    ?mjf_solver
    ?mjf_iterations
    ?mjf_noslip_iterations
    ?mjf_mpr_iterations
    ?mjf_disableflags
    ?mjf_enableflags
    ()
  =
  let x = mjOption_allocate () in
  Option.iter (mjOption_set_timestep x) mjf_timestep;
  Option.iter (mjOption_set_apirate x) mjf_apirate;
  Option.iter (mjOption_set_impratio x) mjf_impratio;
  Option.iter (mjOption_set_tolerance x) mjf_tolerance;
  Option.iter (mjOption_set_noslip_tolerance x) mjf_noslip_tolerance;
  Option.iter (mjOption_set_mpr_tolerance x) mjf_mpr_tolerance;
  Option.iter (mjOption_set_gravity x) mjf_gravity;
  Option.iter (mjOption_set_wind x) mjf_wind;
  Option.iter (mjOption_set_magnetic x) mjf_magnetic;
  Option.iter (mjOption_set_density x) mjf_density;
  Option.iter (mjOption_set_viscosity x) mjf_viscosity;
  Option.iter (mjOption_set_o_margin x) mjf_o_margin;
  Option.iter (mjOption_set_o_solref x) mjf_o_solref;
  Option.iter (mjOption_set_o_solimp x) mjf_o_solimp;
  Option.iter (mjOption_set_integrator x) mjf_integrator;
  Option.iter (mjOption_set_collision x) mjf_collision;
  Option.iter (mjOption_set_cone x) mjf_cone;
  Option.iter (mjOption_set_jacobian x) mjf_jacobian;
  Option.iter (mjOption_set_solver x) mjf_solver;
  Option.iter (mjOption_set_iterations x) mjf_iterations;
  Option.iter (mjOption_set_noslip_iterations x) mjf_noslip_iterations;
  Option.iter (mjOption_set_mpr_iterations x) mjf_mpr_iterations;
  Option.iter (mjOption_set_disableflags x) mjf_disableflags;
  Option.iter (mjOption_set_enableflags x) mjf_enableflags;
  x


(** returns mjOption carray from list of mjOption *)
let mjOption_to_carray = Ctypes.CArray.of_list Typs.mjOption

(** allocate fresh mjStatistic struct *)
let mjStatistic_allocate () = Ctypes.(make Typs.mjStatistic)

(** make null mjStatistic struct ptr *)
let mjStatistic_null () = Ctypes.(from_voidp Typs.mjStatistic null)

(** get meaninertia from mjStatistic *)
let mjStatistic_get_meaninertia x = Ctypes.(getf x Typs.mjStatistic_meaninertia)

(** set meaninertia for mjStatistic *)
let mjStatistic_set_meaninertia x y = Ctypes.(setf x Typs.mjStatistic_meaninertia y)

(** get meanmass from mjStatistic *)
let mjStatistic_get_meanmass x = Ctypes.(getf x Typs.mjStatistic_meanmass)

(** set meanmass for mjStatistic *)
let mjStatistic_set_meanmass x y = Ctypes.(setf x Typs.mjStatistic_meanmass y)

(** get meansize from mjStatistic *)
let mjStatistic_get_meansize x = Ctypes.(getf x Typs.mjStatistic_meansize)

(** set meansize for mjStatistic *)
let mjStatistic_set_meansize x y = Ctypes.(setf x Typs.mjStatistic_meansize y)

(** get extent from mjStatistic *)
let mjStatistic_get_extent x = Ctypes.(getf x Typs.mjStatistic_extent)

(** set extent for mjStatistic *)
let mjStatistic_set_extent x y = Ctypes.(setf x Typs.mjStatistic_extent y)

(** get center from mjStatistic *)
let mjStatistic_get_center x = Ctypes.(getf x Typs.mjStatistic_center)

(** set center for mjStatistic *)
let mjStatistic_set_center x y = Ctypes.(setf x Typs.mjStatistic_center y)

(** make mjStatistic struct *)
let mjStatistic_make
    ?mjf_meaninertia
    ?mjf_meanmass
    ?mjf_meansize
    ?mjf_extent
    ?mjf_center
    ()
  =
  let x = mjStatistic_allocate () in
  Option.iter (mjStatistic_set_meaninertia x) mjf_meaninertia;
  Option.iter (mjStatistic_set_meanmass x) mjf_meanmass;
  Option.iter (mjStatistic_set_meansize x) mjf_meansize;
  Option.iter (mjStatistic_set_extent x) mjf_extent;
  Option.iter (mjStatistic_set_center x) mjf_center;
  x


(** returns mjStatistic carray from list of mjStatistic *)
let mjStatistic_to_carray = Ctypes.CArray.of_list Typs.mjStatistic

(** allocate fresh mjModel struct *)
let mjModel_allocate () = Ctypes.(make Typs.mjModel)

(** make null mjModel struct ptr *)
let mjModel_null () = Ctypes.(from_voidp Typs.mjModel null)

(** get nq from mjModel *)
let mjModel_get_nq x = Ctypes.(getf x Typs.mjModel_nq)

(** set nq for mjModel *)
let mjModel_set_nq x y = Ctypes.(setf x Typs.mjModel_nq y)

(** get nv from mjModel *)
let mjModel_get_nv x = Ctypes.(getf x Typs.mjModel_nv)

(** set nv for mjModel *)
let mjModel_set_nv x y = Ctypes.(setf x Typs.mjModel_nv y)

(** get nu from mjModel *)
let mjModel_get_nu x = Ctypes.(getf x Typs.mjModel_nu)

(** set nu for mjModel *)
let mjModel_set_nu x y = Ctypes.(setf x Typs.mjModel_nu y)

(** get na from mjModel *)
let mjModel_get_na x = Ctypes.(getf x Typs.mjModel_na)

(** set na for mjModel *)
let mjModel_set_na x y = Ctypes.(setf x Typs.mjModel_na y)

(** get nbody from mjModel *)
let mjModel_get_nbody x = Ctypes.(getf x Typs.mjModel_nbody)

(** set nbody for mjModel *)
let mjModel_set_nbody x y = Ctypes.(setf x Typs.mjModel_nbody y)

(** get njnt from mjModel *)
let mjModel_get_njnt x = Ctypes.(getf x Typs.mjModel_njnt)

(** set njnt for mjModel *)
let mjModel_set_njnt x y = Ctypes.(setf x Typs.mjModel_njnt y)

(** get ngeom from mjModel *)
let mjModel_get_ngeom x = Ctypes.(getf x Typs.mjModel_ngeom)

(** set ngeom for mjModel *)
let mjModel_set_ngeom x y = Ctypes.(setf x Typs.mjModel_ngeom y)

(** get nsite from mjModel *)
let mjModel_get_nsite x = Ctypes.(getf x Typs.mjModel_nsite)

(** set nsite for mjModel *)
let mjModel_set_nsite x y = Ctypes.(setf x Typs.mjModel_nsite y)

(** get ncam from mjModel *)
let mjModel_get_ncam x = Ctypes.(getf x Typs.mjModel_ncam)

(** set ncam for mjModel *)
let mjModel_set_ncam x y = Ctypes.(setf x Typs.mjModel_ncam y)

(** get nlight from mjModel *)
let mjModel_get_nlight x = Ctypes.(getf x Typs.mjModel_nlight)

(** set nlight for mjModel *)
let mjModel_set_nlight x y = Ctypes.(setf x Typs.mjModel_nlight y)

(** get nmesh from mjModel *)
let mjModel_get_nmesh x = Ctypes.(getf x Typs.mjModel_nmesh)

(** set nmesh for mjModel *)
let mjModel_set_nmesh x y = Ctypes.(setf x Typs.mjModel_nmesh y)

(** get nmeshvert from mjModel *)
let mjModel_get_nmeshvert x = Ctypes.(getf x Typs.mjModel_nmeshvert)

(** set nmeshvert for mjModel *)
let mjModel_set_nmeshvert x y = Ctypes.(setf x Typs.mjModel_nmeshvert y)

(** get nmeshtexvert from mjModel *)
let mjModel_get_nmeshtexvert x = Ctypes.(getf x Typs.mjModel_nmeshtexvert)

(** set nmeshtexvert for mjModel *)
let mjModel_set_nmeshtexvert x y = Ctypes.(setf x Typs.mjModel_nmeshtexvert y)

(** get nmeshface from mjModel *)
let mjModel_get_nmeshface x = Ctypes.(getf x Typs.mjModel_nmeshface)

(** set nmeshface for mjModel *)
let mjModel_set_nmeshface x y = Ctypes.(setf x Typs.mjModel_nmeshface y)

(** get nmeshgraph from mjModel *)
let mjModel_get_nmeshgraph x = Ctypes.(getf x Typs.mjModel_nmeshgraph)

(** set nmeshgraph for mjModel *)
let mjModel_set_nmeshgraph x y = Ctypes.(setf x Typs.mjModel_nmeshgraph y)

(** get nskin from mjModel *)
let mjModel_get_nskin x = Ctypes.(getf x Typs.mjModel_nskin)

(** set nskin for mjModel *)
let mjModel_set_nskin x y = Ctypes.(setf x Typs.mjModel_nskin y)

(** get nskinvert from mjModel *)
let mjModel_get_nskinvert x = Ctypes.(getf x Typs.mjModel_nskinvert)

(** set nskinvert for mjModel *)
let mjModel_set_nskinvert x y = Ctypes.(setf x Typs.mjModel_nskinvert y)

(** get nskintexvert from mjModel *)
let mjModel_get_nskintexvert x = Ctypes.(getf x Typs.mjModel_nskintexvert)

(** set nskintexvert for mjModel *)
let mjModel_set_nskintexvert x y = Ctypes.(setf x Typs.mjModel_nskintexvert y)

(** get nskinface from mjModel *)
let mjModel_get_nskinface x = Ctypes.(getf x Typs.mjModel_nskinface)

(** set nskinface for mjModel *)
let mjModel_set_nskinface x y = Ctypes.(setf x Typs.mjModel_nskinface y)

(** get nskinbone from mjModel *)
let mjModel_get_nskinbone x = Ctypes.(getf x Typs.mjModel_nskinbone)

(** set nskinbone for mjModel *)
let mjModel_set_nskinbone x y = Ctypes.(setf x Typs.mjModel_nskinbone y)

(** get nskinbonevert from mjModel *)
let mjModel_get_nskinbonevert x = Ctypes.(getf x Typs.mjModel_nskinbonevert)

(** set nskinbonevert for mjModel *)
let mjModel_set_nskinbonevert x y = Ctypes.(setf x Typs.mjModel_nskinbonevert y)

(** get nhfield from mjModel *)
let mjModel_get_nhfield x = Ctypes.(getf x Typs.mjModel_nhfield)

(** set nhfield for mjModel *)
let mjModel_set_nhfield x y = Ctypes.(setf x Typs.mjModel_nhfield y)

(** get nhfielddata from mjModel *)
let mjModel_get_nhfielddata x = Ctypes.(getf x Typs.mjModel_nhfielddata)

(** set nhfielddata for mjModel *)
let mjModel_set_nhfielddata x y = Ctypes.(setf x Typs.mjModel_nhfielddata y)

(** get ntex from mjModel *)
let mjModel_get_ntex x = Ctypes.(getf x Typs.mjModel_ntex)

(** set ntex for mjModel *)
let mjModel_set_ntex x y = Ctypes.(setf x Typs.mjModel_ntex y)

(** get ntexdata from mjModel *)
let mjModel_get_ntexdata x = Ctypes.(getf x Typs.mjModel_ntexdata)

(** set ntexdata for mjModel *)
let mjModel_set_ntexdata x y = Ctypes.(setf x Typs.mjModel_ntexdata y)

(** get nmat from mjModel *)
let mjModel_get_nmat x = Ctypes.(getf x Typs.mjModel_nmat)

(** set nmat for mjModel *)
let mjModel_set_nmat x y = Ctypes.(setf x Typs.mjModel_nmat y)

(** get npair from mjModel *)
let mjModel_get_npair x = Ctypes.(getf x Typs.mjModel_npair)

(** set npair for mjModel *)
let mjModel_set_npair x y = Ctypes.(setf x Typs.mjModel_npair y)

(** get nexclude from mjModel *)
let mjModel_get_nexclude x = Ctypes.(getf x Typs.mjModel_nexclude)

(** set nexclude for mjModel *)
let mjModel_set_nexclude x y = Ctypes.(setf x Typs.mjModel_nexclude y)

(** get neq from mjModel *)
let mjModel_get_neq x = Ctypes.(getf x Typs.mjModel_neq)

(** set neq for mjModel *)
let mjModel_set_neq x y = Ctypes.(setf x Typs.mjModel_neq y)

(** get ntendon from mjModel *)
let mjModel_get_ntendon x = Ctypes.(getf x Typs.mjModel_ntendon)

(** set ntendon for mjModel *)
let mjModel_set_ntendon x y = Ctypes.(setf x Typs.mjModel_ntendon y)

(** get nwrap from mjModel *)
let mjModel_get_nwrap x = Ctypes.(getf x Typs.mjModel_nwrap)

(** set nwrap for mjModel *)
let mjModel_set_nwrap x y = Ctypes.(setf x Typs.mjModel_nwrap y)

(** get nsensor from mjModel *)
let mjModel_get_nsensor x = Ctypes.(getf x Typs.mjModel_nsensor)

(** set nsensor for mjModel *)
let mjModel_set_nsensor x y = Ctypes.(setf x Typs.mjModel_nsensor y)

(** get nnumeric from mjModel *)
let mjModel_get_nnumeric x = Ctypes.(getf x Typs.mjModel_nnumeric)

(** set nnumeric for mjModel *)
let mjModel_set_nnumeric x y = Ctypes.(setf x Typs.mjModel_nnumeric y)

(** get nnumericdata from mjModel *)
let mjModel_get_nnumericdata x = Ctypes.(getf x Typs.mjModel_nnumericdata)

(** set nnumericdata for mjModel *)
let mjModel_set_nnumericdata x y = Ctypes.(setf x Typs.mjModel_nnumericdata y)

(** get ntext from mjModel *)
let mjModel_get_ntext x = Ctypes.(getf x Typs.mjModel_ntext)

(** set ntext for mjModel *)
let mjModel_set_ntext x y = Ctypes.(setf x Typs.mjModel_ntext y)

(** get ntextdata from mjModel *)
let mjModel_get_ntextdata x = Ctypes.(getf x Typs.mjModel_ntextdata)

(** set ntextdata for mjModel *)
let mjModel_set_ntextdata x y = Ctypes.(setf x Typs.mjModel_ntextdata y)

(** get ntuple from mjModel *)
let mjModel_get_ntuple x = Ctypes.(getf x Typs.mjModel_ntuple)

(** set ntuple for mjModel *)
let mjModel_set_ntuple x y = Ctypes.(setf x Typs.mjModel_ntuple y)

(** get ntupledata from mjModel *)
let mjModel_get_ntupledata x = Ctypes.(getf x Typs.mjModel_ntupledata)

(** set ntupledata for mjModel *)
let mjModel_set_ntupledata x y = Ctypes.(setf x Typs.mjModel_ntupledata y)

(** get nkey from mjModel *)
let mjModel_get_nkey x = Ctypes.(getf x Typs.mjModel_nkey)

(** set nkey for mjModel *)
let mjModel_set_nkey x y = Ctypes.(setf x Typs.mjModel_nkey y)

(** get nmocap from mjModel *)
let mjModel_get_nmocap x = Ctypes.(getf x Typs.mjModel_nmocap)

(** set nmocap for mjModel *)
let mjModel_set_nmocap x y = Ctypes.(setf x Typs.mjModel_nmocap y)

(** get nuser_body from mjModel *)
let mjModel_get_nuser_body x = Ctypes.(getf x Typs.mjModel_nuser_body)

(** set nuser_body for mjModel *)
let mjModel_set_nuser_body x y = Ctypes.(setf x Typs.mjModel_nuser_body y)

(** get nuser_jnt from mjModel *)
let mjModel_get_nuser_jnt x = Ctypes.(getf x Typs.mjModel_nuser_jnt)

(** set nuser_jnt for mjModel *)
let mjModel_set_nuser_jnt x y = Ctypes.(setf x Typs.mjModel_nuser_jnt y)

(** get nuser_geom from mjModel *)
let mjModel_get_nuser_geom x = Ctypes.(getf x Typs.mjModel_nuser_geom)

(** set nuser_geom for mjModel *)
let mjModel_set_nuser_geom x y = Ctypes.(setf x Typs.mjModel_nuser_geom y)

(** get nuser_site from mjModel *)
let mjModel_get_nuser_site x = Ctypes.(getf x Typs.mjModel_nuser_site)

(** set nuser_site for mjModel *)
let mjModel_set_nuser_site x y = Ctypes.(setf x Typs.mjModel_nuser_site y)

(** get nuser_cam from mjModel *)
let mjModel_get_nuser_cam x = Ctypes.(getf x Typs.mjModel_nuser_cam)

(** set nuser_cam for mjModel *)
let mjModel_set_nuser_cam x y = Ctypes.(setf x Typs.mjModel_nuser_cam y)

(** get nuser_tendon from mjModel *)
let mjModel_get_nuser_tendon x = Ctypes.(getf x Typs.mjModel_nuser_tendon)

(** set nuser_tendon for mjModel *)
let mjModel_set_nuser_tendon x y = Ctypes.(setf x Typs.mjModel_nuser_tendon y)

(** get nuser_actuator from mjModel *)
let mjModel_get_nuser_actuator x = Ctypes.(getf x Typs.mjModel_nuser_actuator)

(** set nuser_actuator for mjModel *)
let mjModel_set_nuser_actuator x y = Ctypes.(setf x Typs.mjModel_nuser_actuator y)

(** get nuser_sensor from mjModel *)
let mjModel_get_nuser_sensor x = Ctypes.(getf x Typs.mjModel_nuser_sensor)

(** set nuser_sensor for mjModel *)
let mjModel_set_nuser_sensor x y = Ctypes.(setf x Typs.mjModel_nuser_sensor y)

(** get nnames from mjModel *)
let mjModel_get_nnames x = Ctypes.(getf x Typs.mjModel_nnames)

(** set nnames for mjModel *)
let mjModel_set_nnames x y = Ctypes.(setf x Typs.mjModel_nnames y)

(** get nM from mjModel *)
let mjModel_get_nM x = Ctypes.(getf x Typs.mjModel_nM)

(** set nM for mjModel *)
let mjModel_set_nM x y = Ctypes.(setf x Typs.mjModel_nM y)

(** get nemax from mjModel *)
let mjModel_get_nemax x = Ctypes.(getf x Typs.mjModel_nemax)

(** set nemax for mjModel *)
let mjModel_set_nemax x y = Ctypes.(setf x Typs.mjModel_nemax y)

(** get njmax from mjModel *)
let mjModel_get_njmax x = Ctypes.(getf x Typs.mjModel_njmax)

(** set njmax for mjModel *)
let mjModel_set_njmax x y = Ctypes.(setf x Typs.mjModel_njmax y)

(** get nconmax from mjModel *)
let mjModel_get_nconmax x = Ctypes.(getf x Typs.mjModel_nconmax)

(** set nconmax for mjModel *)
let mjModel_set_nconmax x y = Ctypes.(setf x Typs.mjModel_nconmax y)

(** get nstack from mjModel *)
let mjModel_get_nstack x = Ctypes.(getf x Typs.mjModel_nstack)

(** set nstack for mjModel *)
let mjModel_set_nstack x y = Ctypes.(setf x Typs.mjModel_nstack y)

(** get nuserdata from mjModel *)
let mjModel_get_nuserdata x = Ctypes.(getf x Typs.mjModel_nuserdata)

(** set nuserdata for mjModel *)
let mjModel_set_nuserdata x y = Ctypes.(setf x Typs.mjModel_nuserdata y)

(** get nsensordata from mjModel *)
let mjModel_get_nsensordata x = Ctypes.(getf x Typs.mjModel_nsensordata)

(** set nsensordata for mjModel *)
let mjModel_set_nsensordata x y = Ctypes.(setf x Typs.mjModel_nsensordata y)

(** get nbuffer from mjModel *)
let mjModel_get_nbuffer x = Ctypes.(getf x Typs.mjModel_nbuffer)

(** set nbuffer for mjModel *)
let mjModel_set_nbuffer x y = Ctypes.(setf x Typs.mjModel_nbuffer y)

(** get opt from mjModel *)
let mjModel_get_opt x = Ctypes.(getf x Typs.mjModel_opt)

(** set opt for mjModel *)
let mjModel_set_opt x y = Ctypes.(setf x Typs.mjModel_opt y)

(** get vis from mjModel *)
let mjModel_get_vis x = Ctypes.(getf x Typs.mjModel_vis)

(** set vis for mjModel *)
let mjModel_set_vis x y = Ctypes.(setf x Typs.mjModel_vis y)

(** get stat from mjModel *)
let mjModel_get_stat x = Ctypes.(getf x Typs.mjModel_stat)

(** set stat for mjModel *)
let mjModel_set_stat x y = Ctypes.(setf x Typs.mjModel_stat y)

(** get buffer from mjModel *)
let mjModel_get_buffer x = Ctypes.(getf x Typs.mjModel_buffer)

(** set buffer for mjModel *)
let mjModel_set_buffer x y = Ctypes.(setf x Typs.mjModel_buffer y)

(** get qpos0 from mjModel *)
let mjModel_get_qpos0 x = Ctypes.(getf x Typs.mjModel_qpos0)

(** set qpos0 for mjModel *)
let mjModel_set_qpos0 x y = Ctypes.(setf x Typs.mjModel_qpos0 y)

(** get qpos_spring from mjModel *)
let mjModel_get_qpos_spring x = Ctypes.(getf x Typs.mjModel_qpos_spring)

(** set qpos_spring for mjModel *)
let mjModel_set_qpos_spring x y = Ctypes.(setf x Typs.mjModel_qpos_spring y)

(** get body_parentid from mjModel *)
let mjModel_get_body_parentid x = Ctypes.(getf x Typs.mjModel_body_parentid)

(** set body_parentid for mjModel *)
let mjModel_set_body_parentid x y = Ctypes.(setf x Typs.mjModel_body_parentid y)

(** get body_rootid from mjModel *)
let mjModel_get_body_rootid x = Ctypes.(getf x Typs.mjModel_body_rootid)

(** set body_rootid for mjModel *)
let mjModel_set_body_rootid x y = Ctypes.(setf x Typs.mjModel_body_rootid y)

(** get body_weldid from mjModel *)
let mjModel_get_body_weldid x = Ctypes.(getf x Typs.mjModel_body_weldid)

(** set body_weldid for mjModel *)
let mjModel_set_body_weldid x y = Ctypes.(setf x Typs.mjModel_body_weldid y)

(** get body_mocapid from mjModel *)
let mjModel_get_body_mocapid x = Ctypes.(getf x Typs.mjModel_body_mocapid)

(** set body_mocapid for mjModel *)
let mjModel_set_body_mocapid x y = Ctypes.(setf x Typs.mjModel_body_mocapid y)

(** get body_jntnum from mjModel *)
let mjModel_get_body_jntnum x = Ctypes.(getf x Typs.mjModel_body_jntnum)

(** set body_jntnum for mjModel *)
let mjModel_set_body_jntnum x y = Ctypes.(setf x Typs.mjModel_body_jntnum y)

(** get body_jntadr from mjModel *)
let mjModel_get_body_jntadr x = Ctypes.(getf x Typs.mjModel_body_jntadr)

(** set body_jntadr for mjModel *)
let mjModel_set_body_jntadr x y = Ctypes.(setf x Typs.mjModel_body_jntadr y)

(** get body_dofnum from mjModel *)
let mjModel_get_body_dofnum x = Ctypes.(getf x Typs.mjModel_body_dofnum)

(** set body_dofnum for mjModel *)
let mjModel_set_body_dofnum x y = Ctypes.(setf x Typs.mjModel_body_dofnum y)

(** get body_dofadr from mjModel *)
let mjModel_get_body_dofadr x = Ctypes.(getf x Typs.mjModel_body_dofadr)

(** set body_dofadr for mjModel *)
let mjModel_set_body_dofadr x y = Ctypes.(setf x Typs.mjModel_body_dofadr y)

(** get body_geomnum from mjModel *)
let mjModel_get_body_geomnum x = Ctypes.(getf x Typs.mjModel_body_geomnum)

(** set body_geomnum for mjModel *)
let mjModel_set_body_geomnum x y = Ctypes.(setf x Typs.mjModel_body_geomnum y)

(** get body_geomadr from mjModel *)
let mjModel_get_body_geomadr x = Ctypes.(getf x Typs.mjModel_body_geomadr)

(** set body_geomadr for mjModel *)
let mjModel_set_body_geomadr x y = Ctypes.(setf x Typs.mjModel_body_geomadr y)

(** get body_simple from mjModel *)
let mjModel_get_body_simple x = Ctypes.(getf x Typs.mjModel_body_simple)

(** set body_simple for mjModel *)
let mjModel_set_body_simple x y = Ctypes.(setf x Typs.mjModel_body_simple y)

(** get body_sameframe from mjModel *)
let mjModel_get_body_sameframe x = Ctypes.(getf x Typs.mjModel_body_sameframe)

(** set body_sameframe for mjModel *)
let mjModel_set_body_sameframe x y = Ctypes.(setf x Typs.mjModel_body_sameframe y)

(** get body_pos from mjModel *)
let mjModel_get_body_pos x = Ctypes.(getf x Typs.mjModel_body_pos)

(** set body_pos for mjModel *)
let mjModel_set_body_pos x y = Ctypes.(setf x Typs.mjModel_body_pos y)

(** get body_quat from mjModel *)
let mjModel_get_body_quat x = Ctypes.(getf x Typs.mjModel_body_quat)

(** set body_quat for mjModel *)
let mjModel_set_body_quat x y = Ctypes.(setf x Typs.mjModel_body_quat y)

(** get body_ipos from mjModel *)
let mjModel_get_body_ipos x = Ctypes.(getf x Typs.mjModel_body_ipos)

(** set body_ipos for mjModel *)
let mjModel_set_body_ipos x y = Ctypes.(setf x Typs.mjModel_body_ipos y)

(** get body_iquat from mjModel *)
let mjModel_get_body_iquat x = Ctypes.(getf x Typs.mjModel_body_iquat)

(** set body_iquat for mjModel *)
let mjModel_set_body_iquat x y = Ctypes.(setf x Typs.mjModel_body_iquat y)

(** get body_mass from mjModel *)
let mjModel_get_body_mass x = Ctypes.(getf x Typs.mjModel_body_mass)

(** set body_mass for mjModel *)
let mjModel_set_body_mass x y = Ctypes.(setf x Typs.mjModel_body_mass y)

(** get body_subtreemass from mjModel *)
let mjModel_get_body_subtreemass x = Ctypes.(getf x Typs.mjModel_body_subtreemass)

(** set body_subtreemass for mjModel *)
let mjModel_set_body_subtreemass x y = Ctypes.(setf x Typs.mjModel_body_subtreemass y)

(** get body_inertia from mjModel *)
let mjModel_get_body_inertia x = Ctypes.(getf x Typs.mjModel_body_inertia)

(** set body_inertia for mjModel *)
let mjModel_set_body_inertia x y = Ctypes.(setf x Typs.mjModel_body_inertia y)

(** get body_invweight0 from mjModel *)
let mjModel_get_body_invweight0 x = Ctypes.(getf x Typs.mjModel_body_invweight0)

(** set body_invweight0 for mjModel *)
let mjModel_set_body_invweight0 x y = Ctypes.(setf x Typs.mjModel_body_invweight0 y)

(** get body_user from mjModel *)
let mjModel_get_body_user x = Ctypes.(getf x Typs.mjModel_body_user)

(** set body_user for mjModel *)
let mjModel_set_body_user x y = Ctypes.(setf x Typs.mjModel_body_user y)

(** get jnt_type from mjModel *)
let mjModel_get_jnt_type x = Ctypes.(getf x Typs.mjModel_jnt_type)

(** set jnt_type for mjModel *)
let mjModel_set_jnt_type x y = Ctypes.(setf x Typs.mjModel_jnt_type y)

(** get jnt_qposadr from mjModel *)
let mjModel_get_jnt_qposadr x = Ctypes.(getf x Typs.mjModel_jnt_qposadr)

(** set jnt_qposadr for mjModel *)
let mjModel_set_jnt_qposadr x y = Ctypes.(setf x Typs.mjModel_jnt_qposadr y)

(** get jnt_dofadr from mjModel *)
let mjModel_get_jnt_dofadr x = Ctypes.(getf x Typs.mjModel_jnt_dofadr)

(** set jnt_dofadr for mjModel *)
let mjModel_set_jnt_dofadr x y = Ctypes.(setf x Typs.mjModel_jnt_dofadr y)

(** get jnt_bodyid from mjModel *)
let mjModel_get_jnt_bodyid x = Ctypes.(getf x Typs.mjModel_jnt_bodyid)

(** set jnt_bodyid for mjModel *)
let mjModel_set_jnt_bodyid x y = Ctypes.(setf x Typs.mjModel_jnt_bodyid y)

(** get jnt_group from mjModel *)
let mjModel_get_jnt_group x = Ctypes.(getf x Typs.mjModel_jnt_group)

(** set jnt_group for mjModel *)
let mjModel_set_jnt_group x y = Ctypes.(setf x Typs.mjModel_jnt_group y)

(** get jnt_limited from mjModel *)
let mjModel_get_jnt_limited x = Ctypes.(getf x Typs.mjModel_jnt_limited)

(** set jnt_limited for mjModel *)
let mjModel_set_jnt_limited x y = Ctypes.(setf x Typs.mjModel_jnt_limited y)

(** get jnt_solref from mjModel *)
let mjModel_get_jnt_solref x = Ctypes.(getf x Typs.mjModel_jnt_solref)

(** set jnt_solref for mjModel *)
let mjModel_set_jnt_solref x y = Ctypes.(setf x Typs.mjModel_jnt_solref y)

(** get jnt_solimp from mjModel *)
let mjModel_get_jnt_solimp x = Ctypes.(getf x Typs.mjModel_jnt_solimp)

(** set jnt_solimp for mjModel *)
let mjModel_set_jnt_solimp x y = Ctypes.(setf x Typs.mjModel_jnt_solimp y)

(** get jnt_pos from mjModel *)
let mjModel_get_jnt_pos x = Ctypes.(getf x Typs.mjModel_jnt_pos)

(** set jnt_pos for mjModel *)
let mjModel_set_jnt_pos x y = Ctypes.(setf x Typs.mjModel_jnt_pos y)

(** get jnt_axis from mjModel *)
let mjModel_get_jnt_axis x = Ctypes.(getf x Typs.mjModel_jnt_axis)

(** set jnt_axis for mjModel *)
let mjModel_set_jnt_axis x y = Ctypes.(setf x Typs.mjModel_jnt_axis y)

(** get jnt_stiffness from mjModel *)
let mjModel_get_jnt_stiffness x = Ctypes.(getf x Typs.mjModel_jnt_stiffness)

(** set jnt_stiffness for mjModel *)
let mjModel_set_jnt_stiffness x y = Ctypes.(setf x Typs.mjModel_jnt_stiffness y)

(** get jnt_range from mjModel *)
let mjModel_get_jnt_range x = Ctypes.(getf x Typs.mjModel_jnt_range)

(** set jnt_range for mjModel *)
let mjModel_set_jnt_range x y = Ctypes.(setf x Typs.mjModel_jnt_range y)

(** get jnt_margin from mjModel *)
let mjModel_get_jnt_margin x = Ctypes.(getf x Typs.mjModel_jnt_margin)

(** set jnt_margin for mjModel *)
let mjModel_set_jnt_margin x y = Ctypes.(setf x Typs.mjModel_jnt_margin y)

(** get jnt_user from mjModel *)
let mjModel_get_jnt_user x = Ctypes.(getf x Typs.mjModel_jnt_user)

(** set jnt_user for mjModel *)
let mjModel_set_jnt_user x y = Ctypes.(setf x Typs.mjModel_jnt_user y)

(** get dof_bodyid from mjModel *)
let mjModel_get_dof_bodyid x = Ctypes.(getf x Typs.mjModel_dof_bodyid)

(** set dof_bodyid for mjModel *)
let mjModel_set_dof_bodyid x y = Ctypes.(setf x Typs.mjModel_dof_bodyid y)

(** get dof_jntid from mjModel *)
let mjModel_get_dof_jntid x = Ctypes.(getf x Typs.mjModel_dof_jntid)

(** set dof_jntid for mjModel *)
let mjModel_set_dof_jntid x y = Ctypes.(setf x Typs.mjModel_dof_jntid y)

(** get dof_parentid from mjModel *)
let mjModel_get_dof_parentid x = Ctypes.(getf x Typs.mjModel_dof_parentid)

(** set dof_parentid for mjModel *)
let mjModel_set_dof_parentid x y = Ctypes.(setf x Typs.mjModel_dof_parentid y)

(** get dof_Madr from mjModel *)
let mjModel_get_dof_Madr x = Ctypes.(getf x Typs.mjModel_dof_Madr)

(** set dof_Madr for mjModel *)
let mjModel_set_dof_Madr x y = Ctypes.(setf x Typs.mjModel_dof_Madr y)

(** get dof_simplenum from mjModel *)
let mjModel_get_dof_simplenum x = Ctypes.(getf x Typs.mjModel_dof_simplenum)

(** set dof_simplenum for mjModel *)
let mjModel_set_dof_simplenum x y = Ctypes.(setf x Typs.mjModel_dof_simplenum y)

(** get dof_solref from mjModel *)
let mjModel_get_dof_solref x = Ctypes.(getf x Typs.mjModel_dof_solref)

(** set dof_solref for mjModel *)
let mjModel_set_dof_solref x y = Ctypes.(setf x Typs.mjModel_dof_solref y)

(** get dof_solimp from mjModel *)
let mjModel_get_dof_solimp x = Ctypes.(getf x Typs.mjModel_dof_solimp)

(** set dof_solimp for mjModel *)
let mjModel_set_dof_solimp x y = Ctypes.(setf x Typs.mjModel_dof_solimp y)

(** get dof_frictionloss from mjModel *)
let mjModel_get_dof_frictionloss x = Ctypes.(getf x Typs.mjModel_dof_frictionloss)

(** set dof_frictionloss for mjModel *)
let mjModel_set_dof_frictionloss x y = Ctypes.(setf x Typs.mjModel_dof_frictionloss y)

(** get dof_armature from mjModel *)
let mjModel_get_dof_armature x = Ctypes.(getf x Typs.mjModel_dof_armature)

(** set dof_armature for mjModel *)
let mjModel_set_dof_armature x y = Ctypes.(setf x Typs.mjModel_dof_armature y)

(** get dof_damping from mjModel *)
let mjModel_get_dof_damping x = Ctypes.(getf x Typs.mjModel_dof_damping)

(** set dof_damping for mjModel *)
let mjModel_set_dof_damping x y = Ctypes.(setf x Typs.mjModel_dof_damping y)

(** get dof_invweight0 from mjModel *)
let mjModel_get_dof_invweight0 x = Ctypes.(getf x Typs.mjModel_dof_invweight0)

(** set dof_invweight0 for mjModel *)
let mjModel_set_dof_invweight0 x y = Ctypes.(setf x Typs.mjModel_dof_invweight0 y)

(** get dof_M0 from mjModel *)
let mjModel_get_dof_M0 x = Ctypes.(getf x Typs.mjModel_dof_M0)

(** set dof_M0 for mjModel *)
let mjModel_set_dof_M0 x y = Ctypes.(setf x Typs.mjModel_dof_M0 y)

(** get geom_type from mjModel *)
let mjModel_get_geom_type x = Ctypes.(getf x Typs.mjModel_geom_type)

(** set geom_type for mjModel *)
let mjModel_set_geom_type x y = Ctypes.(setf x Typs.mjModel_geom_type y)

(** get geom_contype from mjModel *)
let mjModel_get_geom_contype x = Ctypes.(getf x Typs.mjModel_geom_contype)

(** set geom_contype for mjModel *)
let mjModel_set_geom_contype x y = Ctypes.(setf x Typs.mjModel_geom_contype y)

(** get geom_conaffinity from mjModel *)
let mjModel_get_geom_conaffinity x = Ctypes.(getf x Typs.mjModel_geom_conaffinity)

(** set geom_conaffinity for mjModel *)
let mjModel_set_geom_conaffinity x y = Ctypes.(setf x Typs.mjModel_geom_conaffinity y)

(** get geom_condim from mjModel *)
let mjModel_get_geom_condim x = Ctypes.(getf x Typs.mjModel_geom_condim)

(** set geom_condim for mjModel *)
let mjModel_set_geom_condim x y = Ctypes.(setf x Typs.mjModel_geom_condim y)

(** get geom_bodyid from mjModel *)
let mjModel_get_geom_bodyid x = Ctypes.(getf x Typs.mjModel_geom_bodyid)

(** set geom_bodyid for mjModel *)
let mjModel_set_geom_bodyid x y = Ctypes.(setf x Typs.mjModel_geom_bodyid y)

(** get geom_dataid from mjModel *)
let mjModel_get_geom_dataid x = Ctypes.(getf x Typs.mjModel_geom_dataid)

(** set geom_dataid for mjModel *)
let mjModel_set_geom_dataid x y = Ctypes.(setf x Typs.mjModel_geom_dataid y)

(** get geom_matid from mjModel *)
let mjModel_get_geom_matid x = Ctypes.(getf x Typs.mjModel_geom_matid)

(** set geom_matid for mjModel *)
let mjModel_set_geom_matid x y = Ctypes.(setf x Typs.mjModel_geom_matid y)

(** get geom_group from mjModel *)
let mjModel_get_geom_group x = Ctypes.(getf x Typs.mjModel_geom_group)

(** set geom_group for mjModel *)
let mjModel_set_geom_group x y = Ctypes.(setf x Typs.mjModel_geom_group y)

(** get geom_priority from mjModel *)
let mjModel_get_geom_priority x = Ctypes.(getf x Typs.mjModel_geom_priority)

(** set geom_priority for mjModel *)
let mjModel_set_geom_priority x y = Ctypes.(setf x Typs.mjModel_geom_priority y)

(** get geom_sameframe from mjModel *)
let mjModel_get_geom_sameframe x = Ctypes.(getf x Typs.mjModel_geom_sameframe)

(** set geom_sameframe for mjModel *)
let mjModel_set_geom_sameframe x y = Ctypes.(setf x Typs.mjModel_geom_sameframe y)

(** get geom_solmix from mjModel *)
let mjModel_get_geom_solmix x = Ctypes.(getf x Typs.mjModel_geom_solmix)

(** set geom_solmix for mjModel *)
let mjModel_set_geom_solmix x y = Ctypes.(setf x Typs.mjModel_geom_solmix y)

(** get geom_solref from mjModel *)
let mjModel_get_geom_solref x = Ctypes.(getf x Typs.mjModel_geom_solref)

(** set geom_solref for mjModel *)
let mjModel_set_geom_solref x y = Ctypes.(setf x Typs.mjModel_geom_solref y)

(** get geom_solimp from mjModel *)
let mjModel_get_geom_solimp x = Ctypes.(getf x Typs.mjModel_geom_solimp)

(** set geom_solimp for mjModel *)
let mjModel_set_geom_solimp x y = Ctypes.(setf x Typs.mjModel_geom_solimp y)

(** get geom_size from mjModel *)
let mjModel_get_geom_size x = Ctypes.(getf x Typs.mjModel_geom_size)

(** set geom_size for mjModel *)
let mjModel_set_geom_size x y = Ctypes.(setf x Typs.mjModel_geom_size y)

(** get geom_rbound from mjModel *)
let mjModel_get_geom_rbound x = Ctypes.(getf x Typs.mjModel_geom_rbound)

(** set geom_rbound for mjModel *)
let mjModel_set_geom_rbound x y = Ctypes.(setf x Typs.mjModel_geom_rbound y)

(** get geom_pos from mjModel *)
let mjModel_get_geom_pos x = Ctypes.(getf x Typs.mjModel_geom_pos)

(** set geom_pos for mjModel *)
let mjModel_set_geom_pos x y = Ctypes.(setf x Typs.mjModel_geom_pos y)

(** get geom_quat from mjModel *)
let mjModel_get_geom_quat x = Ctypes.(getf x Typs.mjModel_geom_quat)

(** set geom_quat for mjModel *)
let mjModel_set_geom_quat x y = Ctypes.(setf x Typs.mjModel_geom_quat y)

(** get geom_friction from mjModel *)
let mjModel_get_geom_friction x = Ctypes.(getf x Typs.mjModel_geom_friction)

(** set geom_friction for mjModel *)
let mjModel_set_geom_friction x y = Ctypes.(setf x Typs.mjModel_geom_friction y)

(** get geom_margin from mjModel *)
let mjModel_get_geom_margin x = Ctypes.(getf x Typs.mjModel_geom_margin)

(** set geom_margin for mjModel *)
let mjModel_set_geom_margin x y = Ctypes.(setf x Typs.mjModel_geom_margin y)

(** get geom_gap from mjModel *)
let mjModel_get_geom_gap x = Ctypes.(getf x Typs.mjModel_geom_gap)

(** set geom_gap for mjModel *)
let mjModel_set_geom_gap x y = Ctypes.(setf x Typs.mjModel_geom_gap y)

(** get geom_user from mjModel *)
let mjModel_get_geom_user x = Ctypes.(getf x Typs.mjModel_geom_user)

(** set geom_user for mjModel *)
let mjModel_set_geom_user x y = Ctypes.(setf x Typs.mjModel_geom_user y)

(** get geom_rgba from mjModel *)
let mjModel_get_geom_rgba x = Ctypes.(getf x Typs.mjModel_geom_rgba)

(** set geom_rgba for mjModel *)
let mjModel_set_geom_rgba x y = Ctypes.(setf x Typs.mjModel_geom_rgba y)

(** get site_type from mjModel *)
let mjModel_get_site_type x = Ctypes.(getf x Typs.mjModel_site_type)

(** set site_type for mjModel *)
let mjModel_set_site_type x y = Ctypes.(setf x Typs.mjModel_site_type y)

(** get site_bodyid from mjModel *)
let mjModel_get_site_bodyid x = Ctypes.(getf x Typs.mjModel_site_bodyid)

(** set site_bodyid for mjModel *)
let mjModel_set_site_bodyid x y = Ctypes.(setf x Typs.mjModel_site_bodyid y)

(** get site_matid from mjModel *)
let mjModel_get_site_matid x = Ctypes.(getf x Typs.mjModel_site_matid)

(** set site_matid for mjModel *)
let mjModel_set_site_matid x y = Ctypes.(setf x Typs.mjModel_site_matid y)

(** get site_group from mjModel *)
let mjModel_get_site_group x = Ctypes.(getf x Typs.mjModel_site_group)

(** set site_group for mjModel *)
let mjModel_set_site_group x y = Ctypes.(setf x Typs.mjModel_site_group y)

(** get site_sameframe from mjModel *)
let mjModel_get_site_sameframe x = Ctypes.(getf x Typs.mjModel_site_sameframe)

(** set site_sameframe for mjModel *)
let mjModel_set_site_sameframe x y = Ctypes.(setf x Typs.mjModel_site_sameframe y)

(** get site_size from mjModel *)
let mjModel_get_site_size x = Ctypes.(getf x Typs.mjModel_site_size)

(** set site_size for mjModel *)
let mjModel_set_site_size x y = Ctypes.(setf x Typs.mjModel_site_size y)

(** get site_pos from mjModel *)
let mjModel_get_site_pos x = Ctypes.(getf x Typs.mjModel_site_pos)

(** set site_pos for mjModel *)
let mjModel_set_site_pos x y = Ctypes.(setf x Typs.mjModel_site_pos y)

(** get site_quat from mjModel *)
let mjModel_get_site_quat x = Ctypes.(getf x Typs.mjModel_site_quat)

(** set site_quat for mjModel *)
let mjModel_set_site_quat x y = Ctypes.(setf x Typs.mjModel_site_quat y)

(** get site_user from mjModel *)
let mjModel_get_site_user x = Ctypes.(getf x Typs.mjModel_site_user)

(** set site_user for mjModel *)
let mjModel_set_site_user x y = Ctypes.(setf x Typs.mjModel_site_user y)

(** get site_rgba from mjModel *)
let mjModel_get_site_rgba x = Ctypes.(getf x Typs.mjModel_site_rgba)

(** set site_rgba for mjModel *)
let mjModel_set_site_rgba x y = Ctypes.(setf x Typs.mjModel_site_rgba y)

(** get cam_mode from mjModel *)
let mjModel_get_cam_mode x = Ctypes.(getf x Typs.mjModel_cam_mode)

(** set cam_mode for mjModel *)
let mjModel_set_cam_mode x y = Ctypes.(setf x Typs.mjModel_cam_mode y)

(** get cam_bodyid from mjModel *)
let mjModel_get_cam_bodyid x = Ctypes.(getf x Typs.mjModel_cam_bodyid)

(** set cam_bodyid for mjModel *)
let mjModel_set_cam_bodyid x y = Ctypes.(setf x Typs.mjModel_cam_bodyid y)

(** get cam_targetbodyid from mjModel *)
let mjModel_get_cam_targetbodyid x = Ctypes.(getf x Typs.mjModel_cam_targetbodyid)

(** set cam_targetbodyid for mjModel *)
let mjModel_set_cam_targetbodyid x y = Ctypes.(setf x Typs.mjModel_cam_targetbodyid y)

(** get cam_pos from mjModel *)
let mjModel_get_cam_pos x = Ctypes.(getf x Typs.mjModel_cam_pos)

(** set cam_pos for mjModel *)
let mjModel_set_cam_pos x y = Ctypes.(setf x Typs.mjModel_cam_pos y)

(** get cam_quat from mjModel *)
let mjModel_get_cam_quat x = Ctypes.(getf x Typs.mjModel_cam_quat)

(** set cam_quat for mjModel *)
let mjModel_set_cam_quat x y = Ctypes.(setf x Typs.mjModel_cam_quat y)

(** get cam_poscom0 from mjModel *)
let mjModel_get_cam_poscom0 x = Ctypes.(getf x Typs.mjModel_cam_poscom0)

(** set cam_poscom0 for mjModel *)
let mjModel_set_cam_poscom0 x y = Ctypes.(setf x Typs.mjModel_cam_poscom0 y)

(** get cam_pos0 from mjModel *)
let mjModel_get_cam_pos0 x = Ctypes.(getf x Typs.mjModel_cam_pos0)

(** set cam_pos0 for mjModel *)
let mjModel_set_cam_pos0 x y = Ctypes.(setf x Typs.mjModel_cam_pos0 y)

(** get cam_mat0 from mjModel *)
let mjModel_get_cam_mat0 x = Ctypes.(getf x Typs.mjModel_cam_mat0)

(** set cam_mat0 for mjModel *)
let mjModel_set_cam_mat0 x y = Ctypes.(setf x Typs.mjModel_cam_mat0 y)

(** get cam_fovy from mjModel *)
let mjModel_get_cam_fovy x = Ctypes.(getf x Typs.mjModel_cam_fovy)

(** set cam_fovy for mjModel *)
let mjModel_set_cam_fovy x y = Ctypes.(setf x Typs.mjModel_cam_fovy y)

(** get cam_ipd from mjModel *)
let mjModel_get_cam_ipd x = Ctypes.(getf x Typs.mjModel_cam_ipd)

(** set cam_ipd for mjModel *)
let mjModel_set_cam_ipd x y = Ctypes.(setf x Typs.mjModel_cam_ipd y)

(** get cam_user from mjModel *)
let mjModel_get_cam_user x = Ctypes.(getf x Typs.mjModel_cam_user)

(** set cam_user for mjModel *)
let mjModel_set_cam_user x y = Ctypes.(setf x Typs.mjModel_cam_user y)

(** get light_mode from mjModel *)
let mjModel_get_light_mode x = Ctypes.(getf x Typs.mjModel_light_mode)

(** set light_mode for mjModel *)
let mjModel_set_light_mode x y = Ctypes.(setf x Typs.mjModel_light_mode y)

(** get light_bodyid from mjModel *)
let mjModel_get_light_bodyid x = Ctypes.(getf x Typs.mjModel_light_bodyid)

(** set light_bodyid for mjModel *)
let mjModel_set_light_bodyid x y = Ctypes.(setf x Typs.mjModel_light_bodyid y)

(** get light_targetbodyid from mjModel *)
let mjModel_get_light_targetbodyid x = Ctypes.(getf x Typs.mjModel_light_targetbodyid)

(** set light_targetbodyid for mjModel *)
let mjModel_set_light_targetbodyid x y = Ctypes.(setf x Typs.mjModel_light_targetbodyid y)

(** get light_directional from mjModel *)
let mjModel_get_light_directional x = Ctypes.(getf x Typs.mjModel_light_directional)

(** set light_directional for mjModel *)
let mjModel_set_light_directional x y = Ctypes.(setf x Typs.mjModel_light_directional y)

(** get light_castshadow from mjModel *)
let mjModel_get_light_castshadow x = Ctypes.(getf x Typs.mjModel_light_castshadow)

(** set light_castshadow for mjModel *)
let mjModel_set_light_castshadow x y = Ctypes.(setf x Typs.mjModel_light_castshadow y)

(** get light_active from mjModel *)
let mjModel_get_light_active x = Ctypes.(getf x Typs.mjModel_light_active)

(** set light_active for mjModel *)
let mjModel_set_light_active x y = Ctypes.(setf x Typs.mjModel_light_active y)

(** get light_pos from mjModel *)
let mjModel_get_light_pos x = Ctypes.(getf x Typs.mjModel_light_pos)

(** set light_pos for mjModel *)
let mjModel_set_light_pos x y = Ctypes.(setf x Typs.mjModel_light_pos y)

(** get light_dir from mjModel *)
let mjModel_get_light_dir x = Ctypes.(getf x Typs.mjModel_light_dir)

(** set light_dir for mjModel *)
let mjModel_set_light_dir x y = Ctypes.(setf x Typs.mjModel_light_dir y)

(** get light_poscom0 from mjModel *)
let mjModel_get_light_poscom0 x = Ctypes.(getf x Typs.mjModel_light_poscom0)

(** set light_poscom0 for mjModel *)
let mjModel_set_light_poscom0 x y = Ctypes.(setf x Typs.mjModel_light_poscom0 y)

(** get light_pos0 from mjModel *)
let mjModel_get_light_pos0 x = Ctypes.(getf x Typs.mjModel_light_pos0)

(** set light_pos0 for mjModel *)
let mjModel_set_light_pos0 x y = Ctypes.(setf x Typs.mjModel_light_pos0 y)

(** get light_dir0 from mjModel *)
let mjModel_get_light_dir0 x = Ctypes.(getf x Typs.mjModel_light_dir0)

(** set light_dir0 for mjModel *)
let mjModel_set_light_dir0 x y = Ctypes.(setf x Typs.mjModel_light_dir0 y)

(** get light_attenuation from mjModel *)
let mjModel_get_light_attenuation x = Ctypes.(getf x Typs.mjModel_light_attenuation)

(** set light_attenuation for mjModel *)
let mjModel_set_light_attenuation x y = Ctypes.(setf x Typs.mjModel_light_attenuation y)

(** get light_cutoff from mjModel *)
let mjModel_get_light_cutoff x = Ctypes.(getf x Typs.mjModel_light_cutoff)

(** set light_cutoff for mjModel *)
let mjModel_set_light_cutoff x y = Ctypes.(setf x Typs.mjModel_light_cutoff y)

(** get light_exponent from mjModel *)
let mjModel_get_light_exponent x = Ctypes.(getf x Typs.mjModel_light_exponent)

(** set light_exponent for mjModel *)
let mjModel_set_light_exponent x y = Ctypes.(setf x Typs.mjModel_light_exponent y)

(** get light_ambient from mjModel *)
let mjModel_get_light_ambient x = Ctypes.(getf x Typs.mjModel_light_ambient)

(** set light_ambient for mjModel *)
let mjModel_set_light_ambient x y = Ctypes.(setf x Typs.mjModel_light_ambient y)

(** get light_diffuse from mjModel *)
let mjModel_get_light_diffuse x = Ctypes.(getf x Typs.mjModel_light_diffuse)

(** set light_diffuse for mjModel *)
let mjModel_set_light_diffuse x y = Ctypes.(setf x Typs.mjModel_light_diffuse y)

(** get light_specular from mjModel *)
let mjModel_get_light_specular x = Ctypes.(getf x Typs.mjModel_light_specular)

(** set light_specular for mjModel *)
let mjModel_set_light_specular x y = Ctypes.(setf x Typs.mjModel_light_specular y)

(** get mesh_vertadr from mjModel *)
let mjModel_get_mesh_vertadr x = Ctypes.(getf x Typs.mjModel_mesh_vertadr)

(** set mesh_vertadr for mjModel *)
let mjModel_set_mesh_vertadr x y = Ctypes.(setf x Typs.mjModel_mesh_vertadr y)

(** get mesh_vertnum from mjModel *)
let mjModel_get_mesh_vertnum x = Ctypes.(getf x Typs.mjModel_mesh_vertnum)

(** set mesh_vertnum for mjModel *)
let mjModel_set_mesh_vertnum x y = Ctypes.(setf x Typs.mjModel_mesh_vertnum y)

(** get mesh_texcoordadr from mjModel *)
let mjModel_get_mesh_texcoordadr x = Ctypes.(getf x Typs.mjModel_mesh_texcoordadr)

(** set mesh_texcoordadr for mjModel *)
let mjModel_set_mesh_texcoordadr x y = Ctypes.(setf x Typs.mjModel_mesh_texcoordadr y)

(** get mesh_faceadr from mjModel *)
let mjModel_get_mesh_faceadr x = Ctypes.(getf x Typs.mjModel_mesh_faceadr)

(** set mesh_faceadr for mjModel *)
let mjModel_set_mesh_faceadr x y = Ctypes.(setf x Typs.mjModel_mesh_faceadr y)

(** get mesh_facenum from mjModel *)
let mjModel_get_mesh_facenum x = Ctypes.(getf x Typs.mjModel_mesh_facenum)

(** set mesh_facenum for mjModel *)
let mjModel_set_mesh_facenum x y = Ctypes.(setf x Typs.mjModel_mesh_facenum y)

(** get mesh_graphadr from mjModel *)
let mjModel_get_mesh_graphadr x = Ctypes.(getf x Typs.mjModel_mesh_graphadr)

(** set mesh_graphadr for mjModel *)
let mjModel_set_mesh_graphadr x y = Ctypes.(setf x Typs.mjModel_mesh_graphadr y)

(** get mesh_vert from mjModel *)
let mjModel_get_mesh_vert x = Ctypes.(getf x Typs.mjModel_mesh_vert)

(** set mesh_vert for mjModel *)
let mjModel_set_mesh_vert x y = Ctypes.(setf x Typs.mjModel_mesh_vert y)

(** get mesh_normal from mjModel *)
let mjModel_get_mesh_normal x = Ctypes.(getf x Typs.mjModel_mesh_normal)

(** set mesh_normal for mjModel *)
let mjModel_set_mesh_normal x y = Ctypes.(setf x Typs.mjModel_mesh_normal y)

(** get mesh_texcoord from mjModel *)
let mjModel_get_mesh_texcoord x = Ctypes.(getf x Typs.mjModel_mesh_texcoord)

(** set mesh_texcoord for mjModel *)
let mjModel_set_mesh_texcoord x y = Ctypes.(setf x Typs.mjModel_mesh_texcoord y)

(** get mesh_face from mjModel *)
let mjModel_get_mesh_face x = Ctypes.(getf x Typs.mjModel_mesh_face)

(** set mesh_face for mjModel *)
let mjModel_set_mesh_face x y = Ctypes.(setf x Typs.mjModel_mesh_face y)

(** get mesh_graph from mjModel *)
let mjModel_get_mesh_graph x = Ctypes.(getf x Typs.mjModel_mesh_graph)

(** set mesh_graph for mjModel *)
let mjModel_set_mesh_graph x y = Ctypes.(setf x Typs.mjModel_mesh_graph y)

(** get skin_matid from mjModel *)
let mjModel_get_skin_matid x = Ctypes.(getf x Typs.mjModel_skin_matid)

(** set skin_matid for mjModel *)
let mjModel_set_skin_matid x y = Ctypes.(setf x Typs.mjModel_skin_matid y)

(** get skin_rgba from mjModel *)
let mjModel_get_skin_rgba x = Ctypes.(getf x Typs.mjModel_skin_rgba)

(** set skin_rgba for mjModel *)
let mjModel_set_skin_rgba x y = Ctypes.(setf x Typs.mjModel_skin_rgba y)

(** get skin_inflate from mjModel *)
let mjModel_get_skin_inflate x = Ctypes.(getf x Typs.mjModel_skin_inflate)

(** set skin_inflate for mjModel *)
let mjModel_set_skin_inflate x y = Ctypes.(setf x Typs.mjModel_skin_inflate y)

(** get skin_vertadr from mjModel *)
let mjModel_get_skin_vertadr x = Ctypes.(getf x Typs.mjModel_skin_vertadr)

(** set skin_vertadr for mjModel *)
let mjModel_set_skin_vertadr x y = Ctypes.(setf x Typs.mjModel_skin_vertadr y)

(** get skin_vertnum from mjModel *)
let mjModel_get_skin_vertnum x = Ctypes.(getf x Typs.mjModel_skin_vertnum)

(** set skin_vertnum for mjModel *)
let mjModel_set_skin_vertnum x y = Ctypes.(setf x Typs.mjModel_skin_vertnum y)

(** get skin_texcoordadr from mjModel *)
let mjModel_get_skin_texcoordadr x = Ctypes.(getf x Typs.mjModel_skin_texcoordadr)

(** set skin_texcoordadr for mjModel *)
let mjModel_set_skin_texcoordadr x y = Ctypes.(setf x Typs.mjModel_skin_texcoordadr y)

(** get skin_faceadr from mjModel *)
let mjModel_get_skin_faceadr x = Ctypes.(getf x Typs.mjModel_skin_faceadr)

(** set skin_faceadr for mjModel *)
let mjModel_set_skin_faceadr x y = Ctypes.(setf x Typs.mjModel_skin_faceadr y)

(** get skin_facenum from mjModel *)
let mjModel_get_skin_facenum x = Ctypes.(getf x Typs.mjModel_skin_facenum)

(** set skin_facenum for mjModel *)
let mjModel_set_skin_facenum x y = Ctypes.(setf x Typs.mjModel_skin_facenum y)

(** get skin_boneadr from mjModel *)
let mjModel_get_skin_boneadr x = Ctypes.(getf x Typs.mjModel_skin_boneadr)

(** set skin_boneadr for mjModel *)
let mjModel_set_skin_boneadr x y = Ctypes.(setf x Typs.mjModel_skin_boneadr y)

(** get skin_bonenum from mjModel *)
let mjModel_get_skin_bonenum x = Ctypes.(getf x Typs.mjModel_skin_bonenum)

(** set skin_bonenum for mjModel *)
let mjModel_set_skin_bonenum x y = Ctypes.(setf x Typs.mjModel_skin_bonenum y)

(** get skin_vert from mjModel *)
let mjModel_get_skin_vert x = Ctypes.(getf x Typs.mjModel_skin_vert)

(** set skin_vert for mjModel *)
let mjModel_set_skin_vert x y = Ctypes.(setf x Typs.mjModel_skin_vert y)

(** get skin_texcoord from mjModel *)
let mjModel_get_skin_texcoord x = Ctypes.(getf x Typs.mjModel_skin_texcoord)

(** set skin_texcoord for mjModel *)
let mjModel_set_skin_texcoord x y = Ctypes.(setf x Typs.mjModel_skin_texcoord y)

(** get skin_face from mjModel *)
let mjModel_get_skin_face x = Ctypes.(getf x Typs.mjModel_skin_face)

(** set skin_face for mjModel *)
let mjModel_set_skin_face x y = Ctypes.(setf x Typs.mjModel_skin_face y)

(** get skin_bonevertadr from mjModel *)
let mjModel_get_skin_bonevertadr x = Ctypes.(getf x Typs.mjModel_skin_bonevertadr)

(** set skin_bonevertadr for mjModel *)
let mjModel_set_skin_bonevertadr x y = Ctypes.(setf x Typs.mjModel_skin_bonevertadr y)

(** get skin_bonevertnum from mjModel *)
let mjModel_get_skin_bonevertnum x = Ctypes.(getf x Typs.mjModel_skin_bonevertnum)

(** set skin_bonevertnum for mjModel *)
let mjModel_set_skin_bonevertnum x y = Ctypes.(setf x Typs.mjModel_skin_bonevertnum y)

(** get skin_bonebindpos from mjModel *)
let mjModel_get_skin_bonebindpos x = Ctypes.(getf x Typs.mjModel_skin_bonebindpos)

(** set skin_bonebindpos for mjModel *)
let mjModel_set_skin_bonebindpos x y = Ctypes.(setf x Typs.mjModel_skin_bonebindpos y)

(** get skin_bonebindquat from mjModel *)
let mjModel_get_skin_bonebindquat x = Ctypes.(getf x Typs.mjModel_skin_bonebindquat)

(** set skin_bonebindquat for mjModel *)
let mjModel_set_skin_bonebindquat x y = Ctypes.(setf x Typs.mjModel_skin_bonebindquat y)

(** get skin_bonebodyid from mjModel *)
let mjModel_get_skin_bonebodyid x = Ctypes.(getf x Typs.mjModel_skin_bonebodyid)

(** set skin_bonebodyid for mjModel *)
let mjModel_set_skin_bonebodyid x y = Ctypes.(setf x Typs.mjModel_skin_bonebodyid y)

(** get skin_bonevertid from mjModel *)
let mjModel_get_skin_bonevertid x = Ctypes.(getf x Typs.mjModel_skin_bonevertid)

(** set skin_bonevertid for mjModel *)
let mjModel_set_skin_bonevertid x y = Ctypes.(setf x Typs.mjModel_skin_bonevertid y)

(** get skin_bonevertweight from mjModel *)
let mjModel_get_skin_bonevertweight x = Ctypes.(getf x Typs.mjModel_skin_bonevertweight)

(** set skin_bonevertweight for mjModel *)
let mjModel_set_skin_bonevertweight x y =
  Ctypes.(setf x Typs.mjModel_skin_bonevertweight y)


(** get hfield_size from mjModel *)
let mjModel_get_hfield_size x = Ctypes.(getf x Typs.mjModel_hfield_size)

(** set hfield_size for mjModel *)
let mjModel_set_hfield_size x y = Ctypes.(setf x Typs.mjModel_hfield_size y)

(** get hfield_nrow from mjModel *)
let mjModel_get_hfield_nrow x = Ctypes.(getf x Typs.mjModel_hfield_nrow)

(** set hfield_nrow for mjModel *)
let mjModel_set_hfield_nrow x y = Ctypes.(setf x Typs.mjModel_hfield_nrow y)

(** get hfield_ncol from mjModel *)
let mjModel_get_hfield_ncol x = Ctypes.(getf x Typs.mjModel_hfield_ncol)

(** set hfield_ncol for mjModel *)
let mjModel_set_hfield_ncol x y = Ctypes.(setf x Typs.mjModel_hfield_ncol y)

(** get hfield_adr from mjModel *)
let mjModel_get_hfield_adr x = Ctypes.(getf x Typs.mjModel_hfield_adr)

(** set hfield_adr for mjModel *)
let mjModel_set_hfield_adr x y = Ctypes.(setf x Typs.mjModel_hfield_adr y)

(** get hfield_data from mjModel *)
let mjModel_get_hfield_data x = Ctypes.(getf x Typs.mjModel_hfield_data)

(** set hfield_data for mjModel *)
let mjModel_set_hfield_data x y = Ctypes.(setf x Typs.mjModel_hfield_data y)

(** get tex_type from mjModel *)
let mjModel_get_tex_type x = Ctypes.(getf x Typs.mjModel_tex_type)

(** set tex_type for mjModel *)
let mjModel_set_tex_type x y = Ctypes.(setf x Typs.mjModel_tex_type y)

(** get tex_height from mjModel *)
let mjModel_get_tex_height x = Ctypes.(getf x Typs.mjModel_tex_height)

(** set tex_height for mjModel *)
let mjModel_set_tex_height x y = Ctypes.(setf x Typs.mjModel_tex_height y)

(** get tex_width from mjModel *)
let mjModel_get_tex_width x = Ctypes.(getf x Typs.mjModel_tex_width)

(** set tex_width for mjModel *)
let mjModel_set_tex_width x y = Ctypes.(setf x Typs.mjModel_tex_width y)

(** get tex_adr from mjModel *)
let mjModel_get_tex_adr x = Ctypes.(getf x Typs.mjModel_tex_adr)

(** set tex_adr for mjModel *)
let mjModel_set_tex_adr x y = Ctypes.(setf x Typs.mjModel_tex_adr y)

(** get tex_rgb from mjModel *)
let mjModel_get_tex_rgb x = Ctypes.(getf x Typs.mjModel_tex_rgb)

(** set tex_rgb for mjModel *)
let mjModel_set_tex_rgb x y = Ctypes.(setf x Typs.mjModel_tex_rgb y)

(** get mat_texid from mjModel *)
let mjModel_get_mat_texid x = Ctypes.(getf x Typs.mjModel_mat_texid)

(** set mat_texid for mjModel *)
let mjModel_set_mat_texid x y = Ctypes.(setf x Typs.mjModel_mat_texid y)

(** get mat_texuniform from mjModel *)
let mjModel_get_mat_texuniform x = Ctypes.(getf x Typs.mjModel_mat_texuniform)

(** set mat_texuniform for mjModel *)
let mjModel_set_mat_texuniform x y = Ctypes.(setf x Typs.mjModel_mat_texuniform y)

(** get mat_texrepeat from mjModel *)
let mjModel_get_mat_texrepeat x = Ctypes.(getf x Typs.mjModel_mat_texrepeat)

(** set mat_texrepeat for mjModel *)
let mjModel_set_mat_texrepeat x y = Ctypes.(setf x Typs.mjModel_mat_texrepeat y)

(** get mat_emission from mjModel *)
let mjModel_get_mat_emission x = Ctypes.(getf x Typs.mjModel_mat_emission)

(** set mat_emission for mjModel *)
let mjModel_set_mat_emission x y = Ctypes.(setf x Typs.mjModel_mat_emission y)

(** get mat_specular from mjModel *)
let mjModel_get_mat_specular x = Ctypes.(getf x Typs.mjModel_mat_specular)

(** set mat_specular for mjModel *)
let mjModel_set_mat_specular x y = Ctypes.(setf x Typs.mjModel_mat_specular y)

(** get mat_shininess from mjModel *)
let mjModel_get_mat_shininess x = Ctypes.(getf x Typs.mjModel_mat_shininess)

(** set mat_shininess for mjModel *)
let mjModel_set_mat_shininess x y = Ctypes.(setf x Typs.mjModel_mat_shininess y)

(** get mat_reflectance from mjModel *)
let mjModel_get_mat_reflectance x = Ctypes.(getf x Typs.mjModel_mat_reflectance)

(** set mat_reflectance for mjModel *)
let mjModel_set_mat_reflectance x y = Ctypes.(setf x Typs.mjModel_mat_reflectance y)

(** get mat_rgba from mjModel *)
let mjModel_get_mat_rgba x = Ctypes.(getf x Typs.mjModel_mat_rgba)

(** set mat_rgba for mjModel *)
let mjModel_set_mat_rgba x y = Ctypes.(setf x Typs.mjModel_mat_rgba y)

(** get pair_dim from mjModel *)
let mjModel_get_pair_dim x = Ctypes.(getf x Typs.mjModel_pair_dim)

(** set pair_dim for mjModel *)
let mjModel_set_pair_dim x y = Ctypes.(setf x Typs.mjModel_pair_dim y)

(** get pair_geom1 from mjModel *)
let mjModel_get_pair_geom1 x = Ctypes.(getf x Typs.mjModel_pair_geom1)

(** set pair_geom1 for mjModel *)
let mjModel_set_pair_geom1 x y = Ctypes.(setf x Typs.mjModel_pair_geom1 y)

(** get pair_geom2 from mjModel *)
let mjModel_get_pair_geom2 x = Ctypes.(getf x Typs.mjModel_pair_geom2)

(** set pair_geom2 for mjModel *)
let mjModel_set_pair_geom2 x y = Ctypes.(setf x Typs.mjModel_pair_geom2 y)

(** get pair_signature from mjModel *)
let mjModel_get_pair_signature x = Ctypes.(getf x Typs.mjModel_pair_signature)

(** set pair_signature for mjModel *)
let mjModel_set_pair_signature x y = Ctypes.(setf x Typs.mjModel_pair_signature y)

(** get pair_solref from mjModel *)
let mjModel_get_pair_solref x = Ctypes.(getf x Typs.mjModel_pair_solref)

(** set pair_solref for mjModel *)
let mjModel_set_pair_solref x y = Ctypes.(setf x Typs.mjModel_pair_solref y)

(** get pair_solimp from mjModel *)
let mjModel_get_pair_solimp x = Ctypes.(getf x Typs.mjModel_pair_solimp)

(** set pair_solimp for mjModel *)
let mjModel_set_pair_solimp x y = Ctypes.(setf x Typs.mjModel_pair_solimp y)

(** get pair_margin from mjModel *)
let mjModel_get_pair_margin x = Ctypes.(getf x Typs.mjModel_pair_margin)

(** set pair_margin for mjModel *)
let mjModel_set_pair_margin x y = Ctypes.(setf x Typs.mjModel_pair_margin y)

(** get pair_gap from mjModel *)
let mjModel_get_pair_gap x = Ctypes.(getf x Typs.mjModel_pair_gap)

(** set pair_gap for mjModel *)
let mjModel_set_pair_gap x y = Ctypes.(setf x Typs.mjModel_pair_gap y)

(** get pair_friction from mjModel *)
let mjModel_get_pair_friction x = Ctypes.(getf x Typs.mjModel_pair_friction)

(** set pair_friction for mjModel *)
let mjModel_set_pair_friction x y = Ctypes.(setf x Typs.mjModel_pair_friction y)

(** get exclude_signature from mjModel *)
let mjModel_get_exclude_signature x = Ctypes.(getf x Typs.mjModel_exclude_signature)

(** set exclude_signature for mjModel *)
let mjModel_set_exclude_signature x y = Ctypes.(setf x Typs.mjModel_exclude_signature y)

(** get eq_type from mjModel *)
let mjModel_get_eq_type x = Ctypes.(getf x Typs.mjModel_eq_type)

(** set eq_type for mjModel *)
let mjModel_set_eq_type x y = Ctypes.(setf x Typs.mjModel_eq_type y)

(** get eq_obj1id from mjModel *)
let mjModel_get_eq_obj1id x = Ctypes.(getf x Typs.mjModel_eq_obj1id)

(** set eq_obj1id for mjModel *)
let mjModel_set_eq_obj1id x y = Ctypes.(setf x Typs.mjModel_eq_obj1id y)

(** get eq_obj2id from mjModel *)
let mjModel_get_eq_obj2id x = Ctypes.(getf x Typs.mjModel_eq_obj2id)

(** set eq_obj2id for mjModel *)
let mjModel_set_eq_obj2id x y = Ctypes.(setf x Typs.mjModel_eq_obj2id y)

(** get eq_active from mjModel *)
let mjModel_get_eq_active x = Ctypes.(getf x Typs.mjModel_eq_active)

(** set eq_active for mjModel *)
let mjModel_set_eq_active x y = Ctypes.(setf x Typs.mjModel_eq_active y)

(** get eq_solref from mjModel *)
let mjModel_get_eq_solref x = Ctypes.(getf x Typs.mjModel_eq_solref)

(** set eq_solref for mjModel *)
let mjModel_set_eq_solref x y = Ctypes.(setf x Typs.mjModel_eq_solref y)

(** get eq_solimp from mjModel *)
let mjModel_get_eq_solimp x = Ctypes.(getf x Typs.mjModel_eq_solimp)

(** set eq_solimp for mjModel *)
let mjModel_set_eq_solimp x y = Ctypes.(setf x Typs.mjModel_eq_solimp y)

(** get eq_data from mjModel *)
let mjModel_get_eq_data x = Ctypes.(getf x Typs.mjModel_eq_data)

(** set eq_data for mjModel *)
let mjModel_set_eq_data x y = Ctypes.(setf x Typs.mjModel_eq_data y)

(** get tendon_adr from mjModel *)
let mjModel_get_tendon_adr x = Ctypes.(getf x Typs.mjModel_tendon_adr)

(** set tendon_adr for mjModel *)
let mjModel_set_tendon_adr x y = Ctypes.(setf x Typs.mjModel_tendon_adr y)

(** get tendon_num from mjModel *)
let mjModel_get_tendon_num x = Ctypes.(getf x Typs.mjModel_tendon_num)

(** set tendon_num for mjModel *)
let mjModel_set_tendon_num x y = Ctypes.(setf x Typs.mjModel_tendon_num y)

(** get tendon_matid from mjModel *)
let mjModel_get_tendon_matid x = Ctypes.(getf x Typs.mjModel_tendon_matid)

(** set tendon_matid for mjModel *)
let mjModel_set_tendon_matid x y = Ctypes.(setf x Typs.mjModel_tendon_matid y)

(** get tendon_group from mjModel *)
let mjModel_get_tendon_group x = Ctypes.(getf x Typs.mjModel_tendon_group)

(** set tendon_group for mjModel *)
let mjModel_set_tendon_group x y = Ctypes.(setf x Typs.mjModel_tendon_group y)

(** get tendon_limited from mjModel *)
let mjModel_get_tendon_limited x = Ctypes.(getf x Typs.mjModel_tendon_limited)

(** set tendon_limited for mjModel *)
let mjModel_set_tendon_limited x y = Ctypes.(setf x Typs.mjModel_tendon_limited y)

(** get tendon_width from mjModel *)
let mjModel_get_tendon_width x = Ctypes.(getf x Typs.mjModel_tendon_width)

(** set tendon_width for mjModel *)
let mjModel_set_tendon_width x y = Ctypes.(setf x Typs.mjModel_tendon_width y)

(** get tendon_solref_lim from mjModel *)
let mjModel_get_tendon_solref_lim x = Ctypes.(getf x Typs.mjModel_tendon_solref_lim)

(** set tendon_solref_lim for mjModel *)
let mjModel_set_tendon_solref_lim x y = Ctypes.(setf x Typs.mjModel_tendon_solref_lim y)

(** get tendon_solimp_lim from mjModel *)
let mjModel_get_tendon_solimp_lim x = Ctypes.(getf x Typs.mjModel_tendon_solimp_lim)

(** set tendon_solimp_lim for mjModel *)
let mjModel_set_tendon_solimp_lim x y = Ctypes.(setf x Typs.mjModel_tendon_solimp_lim y)

(** get tendon_solref_fri from mjModel *)
let mjModel_get_tendon_solref_fri x = Ctypes.(getf x Typs.mjModel_tendon_solref_fri)

(** set tendon_solref_fri for mjModel *)
let mjModel_set_tendon_solref_fri x y = Ctypes.(setf x Typs.mjModel_tendon_solref_fri y)

(** get tendon_solimp_fri from mjModel *)
let mjModel_get_tendon_solimp_fri x = Ctypes.(getf x Typs.mjModel_tendon_solimp_fri)

(** set tendon_solimp_fri for mjModel *)
let mjModel_set_tendon_solimp_fri x y = Ctypes.(setf x Typs.mjModel_tendon_solimp_fri y)

(** get tendon_range from mjModel *)
let mjModel_get_tendon_range x = Ctypes.(getf x Typs.mjModel_tendon_range)

(** set tendon_range for mjModel *)
let mjModel_set_tendon_range x y = Ctypes.(setf x Typs.mjModel_tendon_range y)

(** get tendon_margin from mjModel *)
let mjModel_get_tendon_margin x = Ctypes.(getf x Typs.mjModel_tendon_margin)

(** set tendon_margin for mjModel *)
let mjModel_set_tendon_margin x y = Ctypes.(setf x Typs.mjModel_tendon_margin y)

(** get tendon_stiffness from mjModel *)
let mjModel_get_tendon_stiffness x = Ctypes.(getf x Typs.mjModel_tendon_stiffness)

(** set tendon_stiffness for mjModel *)
let mjModel_set_tendon_stiffness x y = Ctypes.(setf x Typs.mjModel_tendon_stiffness y)

(** get tendon_damping from mjModel *)
let mjModel_get_tendon_damping x = Ctypes.(getf x Typs.mjModel_tendon_damping)

(** set tendon_damping for mjModel *)
let mjModel_set_tendon_damping x y = Ctypes.(setf x Typs.mjModel_tendon_damping y)

(** get tendon_frictionloss from mjModel *)
let mjModel_get_tendon_frictionloss x = Ctypes.(getf x Typs.mjModel_tendon_frictionloss)

(** set tendon_frictionloss for mjModel *)
let mjModel_set_tendon_frictionloss x y =
  Ctypes.(setf x Typs.mjModel_tendon_frictionloss y)


(** get tendon_lengthspring from mjModel *)
let mjModel_get_tendon_lengthspring x = Ctypes.(getf x Typs.mjModel_tendon_lengthspring)

(** set tendon_lengthspring for mjModel *)
let mjModel_set_tendon_lengthspring x y =
  Ctypes.(setf x Typs.mjModel_tendon_lengthspring y)


(** get tendon_length0 from mjModel *)
let mjModel_get_tendon_length0 x = Ctypes.(getf x Typs.mjModel_tendon_length0)

(** set tendon_length0 for mjModel *)
let mjModel_set_tendon_length0 x y = Ctypes.(setf x Typs.mjModel_tendon_length0 y)

(** get tendon_invweight0 from mjModel *)
let mjModel_get_tendon_invweight0 x = Ctypes.(getf x Typs.mjModel_tendon_invweight0)

(** set tendon_invweight0 for mjModel *)
let mjModel_set_tendon_invweight0 x y = Ctypes.(setf x Typs.mjModel_tendon_invweight0 y)

(** get tendon_user from mjModel *)
let mjModel_get_tendon_user x = Ctypes.(getf x Typs.mjModel_tendon_user)

(** set tendon_user for mjModel *)
let mjModel_set_tendon_user x y = Ctypes.(setf x Typs.mjModel_tendon_user y)

(** get tendon_rgba from mjModel *)
let mjModel_get_tendon_rgba x = Ctypes.(getf x Typs.mjModel_tendon_rgba)

(** set tendon_rgba for mjModel *)
let mjModel_set_tendon_rgba x y = Ctypes.(setf x Typs.mjModel_tendon_rgba y)

(** get wrap_type from mjModel *)
let mjModel_get_wrap_type x = Ctypes.(getf x Typs.mjModel_wrap_type)

(** set wrap_type for mjModel *)
let mjModel_set_wrap_type x y = Ctypes.(setf x Typs.mjModel_wrap_type y)

(** get wrap_objid from mjModel *)
let mjModel_get_wrap_objid x = Ctypes.(getf x Typs.mjModel_wrap_objid)

(** set wrap_objid for mjModel *)
let mjModel_set_wrap_objid x y = Ctypes.(setf x Typs.mjModel_wrap_objid y)

(** get wrap_prm from mjModel *)
let mjModel_get_wrap_prm x = Ctypes.(getf x Typs.mjModel_wrap_prm)

(** set wrap_prm for mjModel *)
let mjModel_set_wrap_prm x y = Ctypes.(setf x Typs.mjModel_wrap_prm y)

(** get actuator_trntype from mjModel *)
let mjModel_get_actuator_trntype x = Ctypes.(getf x Typs.mjModel_actuator_trntype)

(** set actuator_trntype for mjModel *)
let mjModel_set_actuator_trntype x y = Ctypes.(setf x Typs.mjModel_actuator_trntype y)

(** get actuator_dyntype from mjModel *)
let mjModel_get_actuator_dyntype x = Ctypes.(getf x Typs.mjModel_actuator_dyntype)

(** set actuator_dyntype for mjModel *)
let mjModel_set_actuator_dyntype x y = Ctypes.(setf x Typs.mjModel_actuator_dyntype y)

(** get actuator_gaintype from mjModel *)
let mjModel_get_actuator_gaintype x = Ctypes.(getf x Typs.mjModel_actuator_gaintype)

(** set actuator_gaintype for mjModel *)
let mjModel_set_actuator_gaintype x y = Ctypes.(setf x Typs.mjModel_actuator_gaintype y)

(** get actuator_biastype from mjModel *)
let mjModel_get_actuator_biastype x = Ctypes.(getf x Typs.mjModel_actuator_biastype)

(** set actuator_biastype for mjModel *)
let mjModel_set_actuator_biastype x y = Ctypes.(setf x Typs.mjModel_actuator_biastype y)

(** get actuator_trnid from mjModel *)
let mjModel_get_actuator_trnid x = Ctypes.(getf x Typs.mjModel_actuator_trnid)

(** set actuator_trnid for mjModel *)
let mjModel_set_actuator_trnid x y = Ctypes.(setf x Typs.mjModel_actuator_trnid y)

(** get actuator_group from mjModel *)
let mjModel_get_actuator_group x = Ctypes.(getf x Typs.mjModel_actuator_group)

(** set actuator_group for mjModel *)
let mjModel_set_actuator_group x y = Ctypes.(setf x Typs.mjModel_actuator_group y)

(** get actuator_ctrllimited from mjModel *)
let mjModel_get_actuator_ctrllimited x = Ctypes.(getf x Typs.mjModel_actuator_ctrllimited)

(** set actuator_ctrllimited for mjModel *)
let mjModel_set_actuator_ctrllimited x y =
  Ctypes.(setf x Typs.mjModel_actuator_ctrllimited y)


(** get actuator_forcelimited from mjModel *)
let mjModel_get_actuator_forcelimited x =
  Ctypes.(getf x Typs.mjModel_actuator_forcelimited)


(** set actuator_forcelimited for mjModel *)
let mjModel_set_actuator_forcelimited x y =
  Ctypes.(setf x Typs.mjModel_actuator_forcelimited y)


(** get actuator_dynprm from mjModel *)
let mjModel_get_actuator_dynprm x = Ctypes.(getf x Typs.mjModel_actuator_dynprm)

(** set actuator_dynprm for mjModel *)
let mjModel_set_actuator_dynprm x y = Ctypes.(setf x Typs.mjModel_actuator_dynprm y)

(** get actuator_gainprm from mjModel *)
let mjModel_get_actuator_gainprm x = Ctypes.(getf x Typs.mjModel_actuator_gainprm)

(** set actuator_gainprm for mjModel *)
let mjModel_set_actuator_gainprm x y = Ctypes.(setf x Typs.mjModel_actuator_gainprm y)

(** get actuator_biasprm from mjModel *)
let mjModel_get_actuator_biasprm x = Ctypes.(getf x Typs.mjModel_actuator_biasprm)

(** set actuator_biasprm for mjModel *)
let mjModel_set_actuator_biasprm x y = Ctypes.(setf x Typs.mjModel_actuator_biasprm y)

(** get actuator_ctrlrange from mjModel *)
let mjModel_get_actuator_ctrlrange x = Ctypes.(getf x Typs.mjModel_actuator_ctrlrange)

(** set actuator_ctrlrange for mjModel *)
let mjModel_set_actuator_ctrlrange x y = Ctypes.(setf x Typs.mjModel_actuator_ctrlrange y)

(** get actuator_forcerange from mjModel *)
let mjModel_get_actuator_forcerange x = Ctypes.(getf x Typs.mjModel_actuator_forcerange)

(** set actuator_forcerange for mjModel *)
let mjModel_set_actuator_forcerange x y =
  Ctypes.(setf x Typs.mjModel_actuator_forcerange y)


(** get actuator_gear from mjModel *)
let mjModel_get_actuator_gear x = Ctypes.(getf x Typs.mjModel_actuator_gear)

(** set actuator_gear for mjModel *)
let mjModel_set_actuator_gear x y = Ctypes.(setf x Typs.mjModel_actuator_gear y)

(** get actuator_cranklength from mjModel *)
let mjModel_get_actuator_cranklength x = Ctypes.(getf x Typs.mjModel_actuator_cranklength)

(** set actuator_cranklength for mjModel *)
let mjModel_set_actuator_cranklength x y =
  Ctypes.(setf x Typs.mjModel_actuator_cranklength y)


(** get actuator_acc0 from mjModel *)
let mjModel_get_actuator_acc0 x = Ctypes.(getf x Typs.mjModel_actuator_acc0)

(** set actuator_acc0 for mjModel *)
let mjModel_set_actuator_acc0 x y = Ctypes.(setf x Typs.mjModel_actuator_acc0 y)

(** get actuator_length0 from mjModel *)
let mjModel_get_actuator_length0 x = Ctypes.(getf x Typs.mjModel_actuator_length0)

(** set actuator_length0 for mjModel *)
let mjModel_set_actuator_length0 x y = Ctypes.(setf x Typs.mjModel_actuator_length0 y)

(** get actuator_lengthrange from mjModel *)
let mjModel_get_actuator_lengthrange x = Ctypes.(getf x Typs.mjModel_actuator_lengthrange)

(** set actuator_lengthrange for mjModel *)
let mjModel_set_actuator_lengthrange x y =
  Ctypes.(setf x Typs.mjModel_actuator_lengthrange y)


(** get actuator_user from mjModel *)
let mjModel_get_actuator_user x = Ctypes.(getf x Typs.mjModel_actuator_user)

(** set actuator_user for mjModel *)
let mjModel_set_actuator_user x y = Ctypes.(setf x Typs.mjModel_actuator_user y)

(** get sensor_type from mjModel *)
let mjModel_get_sensor_type x = Ctypes.(getf x Typs.mjModel_sensor_type)

(** set sensor_type for mjModel *)
let mjModel_set_sensor_type x y = Ctypes.(setf x Typs.mjModel_sensor_type y)

(** get sensor_datatype from mjModel *)
let mjModel_get_sensor_datatype x = Ctypes.(getf x Typs.mjModel_sensor_datatype)

(** set sensor_datatype for mjModel *)
let mjModel_set_sensor_datatype x y = Ctypes.(setf x Typs.mjModel_sensor_datatype y)

(** get sensor_needstage from mjModel *)
let mjModel_get_sensor_needstage x = Ctypes.(getf x Typs.mjModel_sensor_needstage)

(** set sensor_needstage for mjModel *)
let mjModel_set_sensor_needstage x y = Ctypes.(setf x Typs.mjModel_sensor_needstage y)

(** get sensor_objtype from mjModel *)
let mjModel_get_sensor_objtype x = Ctypes.(getf x Typs.mjModel_sensor_objtype)

(** set sensor_objtype for mjModel *)
let mjModel_set_sensor_objtype x y = Ctypes.(setf x Typs.mjModel_sensor_objtype y)

(** get sensor_objid from mjModel *)
let mjModel_get_sensor_objid x = Ctypes.(getf x Typs.mjModel_sensor_objid)

(** set sensor_objid for mjModel *)
let mjModel_set_sensor_objid x y = Ctypes.(setf x Typs.mjModel_sensor_objid y)

(** get sensor_dim from mjModel *)
let mjModel_get_sensor_dim x = Ctypes.(getf x Typs.mjModel_sensor_dim)

(** set sensor_dim for mjModel *)
let mjModel_set_sensor_dim x y = Ctypes.(setf x Typs.mjModel_sensor_dim y)

(** get sensor_adr from mjModel *)
let mjModel_get_sensor_adr x = Ctypes.(getf x Typs.mjModel_sensor_adr)

(** set sensor_adr for mjModel *)
let mjModel_set_sensor_adr x y = Ctypes.(setf x Typs.mjModel_sensor_adr y)

(** get sensor_cutoff from mjModel *)
let mjModel_get_sensor_cutoff x = Ctypes.(getf x Typs.mjModel_sensor_cutoff)

(** set sensor_cutoff for mjModel *)
let mjModel_set_sensor_cutoff x y = Ctypes.(setf x Typs.mjModel_sensor_cutoff y)

(** get sensor_noise from mjModel *)
let mjModel_get_sensor_noise x = Ctypes.(getf x Typs.mjModel_sensor_noise)

(** set sensor_noise for mjModel *)
let mjModel_set_sensor_noise x y = Ctypes.(setf x Typs.mjModel_sensor_noise y)

(** get sensor_user from mjModel *)
let mjModel_get_sensor_user x = Ctypes.(getf x Typs.mjModel_sensor_user)

(** set sensor_user for mjModel *)
let mjModel_set_sensor_user x y = Ctypes.(setf x Typs.mjModel_sensor_user y)

(** get numeric_adr from mjModel *)
let mjModel_get_numeric_adr x = Ctypes.(getf x Typs.mjModel_numeric_adr)

(** set numeric_adr for mjModel *)
let mjModel_set_numeric_adr x y = Ctypes.(setf x Typs.mjModel_numeric_adr y)

(** get numeric_size from mjModel *)
let mjModel_get_numeric_size x = Ctypes.(getf x Typs.mjModel_numeric_size)

(** set numeric_size for mjModel *)
let mjModel_set_numeric_size x y = Ctypes.(setf x Typs.mjModel_numeric_size y)

(** get numeric_data from mjModel *)
let mjModel_get_numeric_data x = Ctypes.(getf x Typs.mjModel_numeric_data)

(** set numeric_data for mjModel *)
let mjModel_set_numeric_data x y = Ctypes.(setf x Typs.mjModel_numeric_data y)

(** get text_adr from mjModel *)
let mjModel_get_text_adr x = Ctypes.(getf x Typs.mjModel_text_adr)

(** set text_adr for mjModel *)
let mjModel_set_text_adr x y = Ctypes.(setf x Typs.mjModel_text_adr y)

(** get text_size from mjModel *)
let mjModel_get_text_size x = Ctypes.(getf x Typs.mjModel_text_size)

(** set text_size for mjModel *)
let mjModel_set_text_size x y = Ctypes.(setf x Typs.mjModel_text_size y)

(** get text_data from mjModel *)
let mjModel_get_text_data x = Ctypes.(getf x Typs.mjModel_text_data)

(** set text_data for mjModel *)
let mjModel_set_text_data x y = Ctypes.(setf x Typs.mjModel_text_data y)

(** get tuple_adr from mjModel *)
let mjModel_get_tuple_adr x = Ctypes.(getf x Typs.mjModel_tuple_adr)

(** set tuple_adr for mjModel *)
let mjModel_set_tuple_adr x y = Ctypes.(setf x Typs.mjModel_tuple_adr y)

(** get tuple_size from mjModel *)
let mjModel_get_tuple_size x = Ctypes.(getf x Typs.mjModel_tuple_size)

(** set tuple_size for mjModel *)
let mjModel_set_tuple_size x y = Ctypes.(setf x Typs.mjModel_tuple_size y)

(** get tuple_objtype from mjModel *)
let mjModel_get_tuple_objtype x = Ctypes.(getf x Typs.mjModel_tuple_objtype)

(** set tuple_objtype for mjModel *)
let mjModel_set_tuple_objtype x y = Ctypes.(setf x Typs.mjModel_tuple_objtype y)

(** get tuple_objid from mjModel *)
let mjModel_get_tuple_objid x = Ctypes.(getf x Typs.mjModel_tuple_objid)

(** set tuple_objid for mjModel *)
let mjModel_set_tuple_objid x y = Ctypes.(setf x Typs.mjModel_tuple_objid y)

(** get tuple_objprm from mjModel *)
let mjModel_get_tuple_objprm x = Ctypes.(getf x Typs.mjModel_tuple_objprm)

(** set tuple_objprm for mjModel *)
let mjModel_set_tuple_objprm x y = Ctypes.(setf x Typs.mjModel_tuple_objprm y)

(** get key_time from mjModel *)
let mjModel_get_key_time x = Ctypes.(getf x Typs.mjModel_key_time)

(** set key_time for mjModel *)
let mjModel_set_key_time x y = Ctypes.(setf x Typs.mjModel_key_time y)

(** get key_qpos from mjModel *)
let mjModel_get_key_qpos x = Ctypes.(getf x Typs.mjModel_key_qpos)

(** set key_qpos for mjModel *)
let mjModel_set_key_qpos x y = Ctypes.(setf x Typs.mjModel_key_qpos y)

(** get key_qvel from mjModel *)
let mjModel_get_key_qvel x = Ctypes.(getf x Typs.mjModel_key_qvel)

(** set key_qvel for mjModel *)
let mjModel_set_key_qvel x y = Ctypes.(setf x Typs.mjModel_key_qvel y)

(** get key_act from mjModel *)
let mjModel_get_key_act x = Ctypes.(getf x Typs.mjModel_key_act)

(** set key_act for mjModel *)
let mjModel_set_key_act x y = Ctypes.(setf x Typs.mjModel_key_act y)

(** get key_mpos from mjModel *)
let mjModel_get_key_mpos x = Ctypes.(getf x Typs.mjModel_key_mpos)

(** set key_mpos for mjModel *)
let mjModel_set_key_mpos x y = Ctypes.(setf x Typs.mjModel_key_mpos y)

(** get key_mquat from mjModel *)
let mjModel_get_key_mquat x = Ctypes.(getf x Typs.mjModel_key_mquat)

(** set key_mquat for mjModel *)
let mjModel_set_key_mquat x y = Ctypes.(setf x Typs.mjModel_key_mquat y)

(** get name_bodyadr from mjModel *)
let mjModel_get_name_bodyadr x = Ctypes.(getf x Typs.mjModel_name_bodyadr)

(** set name_bodyadr for mjModel *)
let mjModel_set_name_bodyadr x y = Ctypes.(setf x Typs.mjModel_name_bodyadr y)

(** get name_jntadr from mjModel *)
let mjModel_get_name_jntadr x = Ctypes.(getf x Typs.mjModel_name_jntadr)

(** set name_jntadr for mjModel *)
let mjModel_set_name_jntadr x y = Ctypes.(setf x Typs.mjModel_name_jntadr y)

(** get name_geomadr from mjModel *)
let mjModel_get_name_geomadr x = Ctypes.(getf x Typs.mjModel_name_geomadr)

(** set name_geomadr for mjModel *)
let mjModel_set_name_geomadr x y = Ctypes.(setf x Typs.mjModel_name_geomadr y)

(** get name_siteadr from mjModel *)
let mjModel_get_name_siteadr x = Ctypes.(getf x Typs.mjModel_name_siteadr)

(** set name_siteadr for mjModel *)
let mjModel_set_name_siteadr x y = Ctypes.(setf x Typs.mjModel_name_siteadr y)

(** get name_camadr from mjModel *)
let mjModel_get_name_camadr x = Ctypes.(getf x Typs.mjModel_name_camadr)

(** set name_camadr for mjModel *)
let mjModel_set_name_camadr x y = Ctypes.(setf x Typs.mjModel_name_camadr y)

(** get name_lightadr from mjModel *)
let mjModel_get_name_lightadr x = Ctypes.(getf x Typs.mjModel_name_lightadr)

(** set name_lightadr for mjModel *)
let mjModel_set_name_lightadr x y = Ctypes.(setf x Typs.mjModel_name_lightadr y)

(** get name_meshadr from mjModel *)
let mjModel_get_name_meshadr x = Ctypes.(getf x Typs.mjModel_name_meshadr)

(** set name_meshadr for mjModel *)
let mjModel_set_name_meshadr x y = Ctypes.(setf x Typs.mjModel_name_meshadr y)

(** get name_skinadr from mjModel *)
let mjModel_get_name_skinadr x = Ctypes.(getf x Typs.mjModel_name_skinadr)

(** set name_skinadr for mjModel *)
let mjModel_set_name_skinadr x y = Ctypes.(setf x Typs.mjModel_name_skinadr y)

(** get name_hfieldadr from mjModel *)
let mjModel_get_name_hfieldadr x = Ctypes.(getf x Typs.mjModel_name_hfieldadr)

(** set name_hfieldadr for mjModel *)
let mjModel_set_name_hfieldadr x y = Ctypes.(setf x Typs.mjModel_name_hfieldadr y)

(** get name_texadr from mjModel *)
let mjModel_get_name_texadr x = Ctypes.(getf x Typs.mjModel_name_texadr)

(** set name_texadr for mjModel *)
let mjModel_set_name_texadr x y = Ctypes.(setf x Typs.mjModel_name_texadr y)

(** get name_matadr from mjModel *)
let mjModel_get_name_matadr x = Ctypes.(getf x Typs.mjModel_name_matadr)

(** set name_matadr for mjModel *)
let mjModel_set_name_matadr x y = Ctypes.(setf x Typs.mjModel_name_matadr y)

(** get name_pairadr from mjModel *)
let mjModel_get_name_pairadr x = Ctypes.(getf x Typs.mjModel_name_pairadr)

(** set name_pairadr for mjModel *)
let mjModel_set_name_pairadr x y = Ctypes.(setf x Typs.mjModel_name_pairadr y)

(** get name_excludeadr from mjModel *)
let mjModel_get_name_excludeadr x = Ctypes.(getf x Typs.mjModel_name_excludeadr)

(** set name_excludeadr for mjModel *)
let mjModel_set_name_excludeadr x y = Ctypes.(setf x Typs.mjModel_name_excludeadr y)

(** get name_eqadr from mjModel *)
let mjModel_get_name_eqadr x = Ctypes.(getf x Typs.mjModel_name_eqadr)

(** set name_eqadr for mjModel *)
let mjModel_set_name_eqadr x y = Ctypes.(setf x Typs.mjModel_name_eqadr y)

(** get name_tendonadr from mjModel *)
let mjModel_get_name_tendonadr x = Ctypes.(getf x Typs.mjModel_name_tendonadr)

(** set name_tendonadr for mjModel *)
let mjModel_set_name_tendonadr x y = Ctypes.(setf x Typs.mjModel_name_tendonadr y)

(** get name_actuatoradr from mjModel *)
let mjModel_get_name_actuatoradr x = Ctypes.(getf x Typs.mjModel_name_actuatoradr)

(** set name_actuatoradr for mjModel *)
let mjModel_set_name_actuatoradr x y = Ctypes.(setf x Typs.mjModel_name_actuatoradr y)

(** get name_sensoradr from mjModel *)
let mjModel_get_name_sensoradr x = Ctypes.(getf x Typs.mjModel_name_sensoradr)

(** set name_sensoradr for mjModel *)
let mjModel_set_name_sensoradr x y = Ctypes.(setf x Typs.mjModel_name_sensoradr y)

(** get name_numericadr from mjModel *)
let mjModel_get_name_numericadr x = Ctypes.(getf x Typs.mjModel_name_numericadr)

(** set name_numericadr for mjModel *)
let mjModel_set_name_numericadr x y = Ctypes.(setf x Typs.mjModel_name_numericadr y)

(** get name_textadr from mjModel *)
let mjModel_get_name_textadr x = Ctypes.(getf x Typs.mjModel_name_textadr)

(** set name_textadr for mjModel *)
let mjModel_set_name_textadr x y = Ctypes.(setf x Typs.mjModel_name_textadr y)

(** get name_tupleadr from mjModel *)
let mjModel_get_name_tupleadr x = Ctypes.(getf x Typs.mjModel_name_tupleadr)

(** set name_tupleadr for mjModel *)
let mjModel_set_name_tupleadr x y = Ctypes.(setf x Typs.mjModel_name_tupleadr y)

(** get name_keyadr from mjModel *)
let mjModel_get_name_keyadr x = Ctypes.(getf x Typs.mjModel_name_keyadr)

(** set name_keyadr for mjModel *)
let mjModel_set_name_keyadr x y = Ctypes.(setf x Typs.mjModel_name_keyadr y)

(** get names from mjModel *)
let mjModel_get_names x = Ctypes.(getf x Typs.mjModel_names)

(** set names for mjModel *)
let mjModel_set_names x y = Ctypes.(setf x Typs.mjModel_names y)

(** make mjModel struct *)
let mjModel_make
    ?mjf_nq
    ?mjf_nv
    ?mjf_nu
    ?mjf_na
    ?mjf_nbody
    ?mjf_njnt
    ?mjf_ngeom
    ?mjf_nsite
    ?mjf_ncam
    ?mjf_nlight
    ?mjf_nmesh
    ?mjf_nmeshvert
    ?mjf_nmeshtexvert
    ?mjf_nmeshface
    ?mjf_nmeshgraph
    ?mjf_nskin
    ?mjf_nskinvert
    ?mjf_nskintexvert
    ?mjf_nskinface
    ?mjf_nskinbone
    ?mjf_nskinbonevert
    ?mjf_nhfield
    ?mjf_nhfielddata
    ?mjf_ntex
    ?mjf_ntexdata
    ?mjf_nmat
    ?mjf_npair
    ?mjf_nexclude
    ?mjf_neq
    ?mjf_ntendon
    ?mjf_nwrap
    ?mjf_nsensor
    ?mjf_nnumeric
    ?mjf_nnumericdata
    ?mjf_ntext
    ?mjf_ntextdata
    ?mjf_ntuple
    ?mjf_ntupledata
    ?mjf_nkey
    ?mjf_nmocap
    ?mjf_nuser_body
    ?mjf_nuser_jnt
    ?mjf_nuser_geom
    ?mjf_nuser_site
    ?mjf_nuser_cam
    ?mjf_nuser_tendon
    ?mjf_nuser_actuator
    ?mjf_nuser_sensor
    ?mjf_nnames
    ?mjf_nM
    ?mjf_nemax
    ?mjf_njmax
    ?mjf_nconmax
    ?mjf_nstack
    ?mjf_nuserdata
    ?mjf_nsensordata
    ?mjf_nbuffer
    ?mjf_opt
    ?mjf_vis
    ?mjf_stat
    ?mjf_buffer
    ?mjf_qpos0
    ?mjf_qpos_spring
    ?mjf_body_parentid
    ?mjf_body_rootid
    ?mjf_body_weldid
    ?mjf_body_mocapid
    ?mjf_body_jntnum
    ?mjf_body_jntadr
    ?mjf_body_dofnum
    ?mjf_body_dofadr
    ?mjf_body_geomnum
    ?mjf_body_geomadr
    ?mjf_body_simple
    ?mjf_body_sameframe
    ?mjf_body_pos
    ?mjf_body_quat
    ?mjf_body_ipos
    ?mjf_body_iquat
    ?mjf_body_mass
    ?mjf_body_subtreemass
    ?mjf_body_inertia
    ?mjf_body_invweight0
    ?mjf_body_user
    ?mjf_jnt_type
    ?mjf_jnt_qposadr
    ?mjf_jnt_dofadr
    ?mjf_jnt_bodyid
    ?mjf_jnt_group
    ?mjf_jnt_limited
    ?mjf_jnt_solref
    ?mjf_jnt_solimp
    ?mjf_jnt_pos
    ?mjf_jnt_axis
    ?mjf_jnt_stiffness
    ?mjf_jnt_range
    ?mjf_jnt_margin
    ?mjf_jnt_user
    ?mjf_dof_bodyid
    ?mjf_dof_jntid
    ?mjf_dof_parentid
    ?mjf_dof_Madr
    ?mjf_dof_simplenum
    ?mjf_dof_solref
    ?mjf_dof_solimp
    ?mjf_dof_frictionloss
    ?mjf_dof_armature
    ?mjf_dof_damping
    ?mjf_dof_invweight0
    ?mjf_dof_M0
    ?mjf_geom_type
    ?mjf_geom_contype
    ?mjf_geom_conaffinity
    ?mjf_geom_condim
    ?mjf_geom_bodyid
    ?mjf_geom_dataid
    ?mjf_geom_matid
    ?mjf_geom_group
    ?mjf_geom_priority
    ?mjf_geom_sameframe
    ?mjf_geom_solmix
    ?mjf_geom_solref
    ?mjf_geom_solimp
    ?mjf_geom_size
    ?mjf_geom_rbound
    ?mjf_geom_pos
    ?mjf_geom_quat
    ?mjf_geom_friction
    ?mjf_geom_margin
    ?mjf_geom_gap
    ?mjf_geom_user
    ?mjf_geom_rgba
    ?mjf_site_type
    ?mjf_site_bodyid
    ?mjf_site_matid
    ?mjf_site_group
    ?mjf_site_sameframe
    ?mjf_site_size
    ?mjf_site_pos
    ?mjf_site_quat
    ?mjf_site_user
    ?mjf_site_rgba
    ?mjf_cam_mode
    ?mjf_cam_bodyid
    ?mjf_cam_targetbodyid
    ?mjf_cam_pos
    ?mjf_cam_quat
    ?mjf_cam_poscom0
    ?mjf_cam_pos0
    ?mjf_cam_mat0
    ?mjf_cam_fovy
    ?mjf_cam_ipd
    ?mjf_cam_user
    ?mjf_light_mode
    ?mjf_light_bodyid
    ?mjf_light_targetbodyid
    ?mjf_light_directional
    ?mjf_light_castshadow
    ?mjf_light_active
    ?mjf_light_pos
    ?mjf_light_dir
    ?mjf_light_poscom0
    ?mjf_light_pos0
    ?mjf_light_dir0
    ?mjf_light_attenuation
    ?mjf_light_cutoff
    ?mjf_light_exponent
    ?mjf_light_ambient
    ?mjf_light_diffuse
    ?mjf_light_specular
    ?mjf_mesh_vertadr
    ?mjf_mesh_vertnum
    ?mjf_mesh_texcoordadr
    ?mjf_mesh_faceadr
    ?mjf_mesh_facenum
    ?mjf_mesh_graphadr
    ?mjf_mesh_vert
    ?mjf_mesh_normal
    ?mjf_mesh_texcoord
    ?mjf_mesh_face
    ?mjf_mesh_graph
    ?mjf_skin_matid
    ?mjf_skin_rgba
    ?mjf_skin_inflate
    ?mjf_skin_vertadr
    ?mjf_skin_vertnum
    ?mjf_skin_texcoordadr
    ?mjf_skin_faceadr
    ?mjf_skin_facenum
    ?mjf_skin_boneadr
    ?mjf_skin_bonenum
    ?mjf_skin_vert
    ?mjf_skin_texcoord
    ?mjf_skin_face
    ?mjf_skin_bonevertadr
    ?mjf_skin_bonevertnum
    ?mjf_skin_bonebindpos
    ?mjf_skin_bonebindquat
    ?mjf_skin_bonebodyid
    ?mjf_skin_bonevertid
    ?mjf_skin_bonevertweight
    ?mjf_hfield_size
    ?mjf_hfield_nrow
    ?mjf_hfield_ncol
    ?mjf_hfield_adr
    ?mjf_hfield_data
    ?mjf_tex_type
    ?mjf_tex_height
    ?mjf_tex_width
    ?mjf_tex_adr
    ?mjf_tex_rgb
    ?mjf_mat_texid
    ?mjf_mat_texuniform
    ?mjf_mat_texrepeat
    ?mjf_mat_emission
    ?mjf_mat_specular
    ?mjf_mat_shininess
    ?mjf_mat_reflectance
    ?mjf_mat_rgba
    ?mjf_pair_dim
    ?mjf_pair_geom1
    ?mjf_pair_geom2
    ?mjf_pair_signature
    ?mjf_pair_solref
    ?mjf_pair_solimp
    ?mjf_pair_margin
    ?mjf_pair_gap
    ?mjf_pair_friction
    ?mjf_exclude_signature
    ?mjf_eq_type
    ?mjf_eq_obj1id
    ?mjf_eq_obj2id
    ?mjf_eq_active
    ?mjf_eq_solref
    ?mjf_eq_solimp
    ?mjf_eq_data
    ?mjf_tendon_adr
    ?mjf_tendon_num
    ?mjf_tendon_matid
    ?mjf_tendon_group
    ?mjf_tendon_limited
    ?mjf_tendon_width
    ?mjf_tendon_solref_lim
    ?mjf_tendon_solimp_lim
    ?mjf_tendon_solref_fri
    ?mjf_tendon_solimp_fri
    ?mjf_tendon_range
    ?mjf_tendon_margin
    ?mjf_tendon_stiffness
    ?mjf_tendon_damping
    ?mjf_tendon_frictionloss
    ?mjf_tendon_lengthspring
    ?mjf_tendon_length0
    ?mjf_tendon_invweight0
    ?mjf_tendon_user
    ?mjf_tendon_rgba
    ?mjf_wrap_type
    ?mjf_wrap_objid
    ?mjf_wrap_prm
    ?mjf_actuator_trntype
    ?mjf_actuator_dyntype
    ?mjf_actuator_gaintype
    ?mjf_actuator_biastype
    ?mjf_actuator_trnid
    ?mjf_actuator_group
    ?mjf_actuator_ctrllimited
    ?mjf_actuator_forcelimited
    ?mjf_actuator_dynprm
    ?mjf_actuator_gainprm
    ?mjf_actuator_biasprm
    ?mjf_actuator_ctrlrange
    ?mjf_actuator_forcerange
    ?mjf_actuator_gear
    ?mjf_actuator_cranklength
    ?mjf_actuator_acc0
    ?mjf_actuator_length0
    ?mjf_actuator_lengthrange
    ?mjf_actuator_user
    ?mjf_sensor_type
    ?mjf_sensor_datatype
    ?mjf_sensor_needstage
    ?mjf_sensor_objtype
    ?mjf_sensor_objid
    ?mjf_sensor_dim
    ?mjf_sensor_adr
    ?mjf_sensor_cutoff
    ?mjf_sensor_noise
    ?mjf_sensor_user
    ?mjf_numeric_adr
    ?mjf_numeric_size
    ?mjf_numeric_data
    ?mjf_text_adr
    ?mjf_text_size
    ?mjf_text_data
    ?mjf_tuple_adr
    ?mjf_tuple_size
    ?mjf_tuple_objtype
    ?mjf_tuple_objid
    ?mjf_tuple_objprm
    ?mjf_key_time
    ?mjf_key_qpos
    ?mjf_key_qvel
    ?mjf_key_act
    ?mjf_key_mpos
    ?mjf_key_mquat
    ?mjf_name_bodyadr
    ?mjf_name_jntadr
    ?mjf_name_geomadr
    ?mjf_name_siteadr
    ?mjf_name_camadr
    ?mjf_name_lightadr
    ?mjf_name_meshadr
    ?mjf_name_skinadr
    ?mjf_name_hfieldadr
    ?mjf_name_texadr
    ?mjf_name_matadr
    ?mjf_name_pairadr
    ?mjf_name_excludeadr
    ?mjf_name_eqadr
    ?mjf_name_tendonadr
    ?mjf_name_actuatoradr
    ?mjf_name_sensoradr
    ?mjf_name_numericadr
    ?mjf_name_textadr
    ?mjf_name_tupleadr
    ?mjf_name_keyadr
    ?mjf_names
    ()
  =
  let x = mjModel_allocate () in
  Option.iter (mjModel_set_nq x) mjf_nq;
  Option.iter (mjModel_set_nv x) mjf_nv;
  Option.iter (mjModel_set_nu x) mjf_nu;
  Option.iter (mjModel_set_na x) mjf_na;
  Option.iter (mjModel_set_nbody x) mjf_nbody;
  Option.iter (mjModel_set_njnt x) mjf_njnt;
  Option.iter (mjModel_set_ngeom x) mjf_ngeom;
  Option.iter (mjModel_set_nsite x) mjf_nsite;
  Option.iter (mjModel_set_ncam x) mjf_ncam;
  Option.iter (mjModel_set_nlight x) mjf_nlight;
  Option.iter (mjModel_set_nmesh x) mjf_nmesh;
  Option.iter (mjModel_set_nmeshvert x) mjf_nmeshvert;
  Option.iter (mjModel_set_nmeshtexvert x) mjf_nmeshtexvert;
  Option.iter (mjModel_set_nmeshface x) mjf_nmeshface;
  Option.iter (mjModel_set_nmeshgraph x) mjf_nmeshgraph;
  Option.iter (mjModel_set_nskin x) mjf_nskin;
  Option.iter (mjModel_set_nskinvert x) mjf_nskinvert;
  Option.iter (mjModel_set_nskintexvert x) mjf_nskintexvert;
  Option.iter (mjModel_set_nskinface x) mjf_nskinface;
  Option.iter (mjModel_set_nskinbone x) mjf_nskinbone;
  Option.iter (mjModel_set_nskinbonevert x) mjf_nskinbonevert;
  Option.iter (mjModel_set_nhfield x) mjf_nhfield;
  Option.iter (mjModel_set_nhfielddata x) mjf_nhfielddata;
  Option.iter (mjModel_set_ntex x) mjf_ntex;
  Option.iter (mjModel_set_ntexdata x) mjf_ntexdata;
  Option.iter (mjModel_set_nmat x) mjf_nmat;
  Option.iter (mjModel_set_npair x) mjf_npair;
  Option.iter (mjModel_set_nexclude x) mjf_nexclude;
  Option.iter (mjModel_set_neq x) mjf_neq;
  Option.iter (mjModel_set_ntendon x) mjf_ntendon;
  Option.iter (mjModel_set_nwrap x) mjf_nwrap;
  Option.iter (mjModel_set_nsensor x) mjf_nsensor;
  Option.iter (mjModel_set_nnumeric x) mjf_nnumeric;
  Option.iter (mjModel_set_nnumericdata x) mjf_nnumericdata;
  Option.iter (mjModel_set_ntext x) mjf_ntext;
  Option.iter (mjModel_set_ntextdata x) mjf_ntextdata;
  Option.iter (mjModel_set_ntuple x) mjf_ntuple;
  Option.iter (mjModel_set_ntupledata x) mjf_ntupledata;
  Option.iter (mjModel_set_nkey x) mjf_nkey;
  Option.iter (mjModel_set_nmocap x) mjf_nmocap;
  Option.iter (mjModel_set_nuser_body x) mjf_nuser_body;
  Option.iter (mjModel_set_nuser_jnt x) mjf_nuser_jnt;
  Option.iter (mjModel_set_nuser_geom x) mjf_nuser_geom;
  Option.iter (mjModel_set_nuser_site x) mjf_nuser_site;
  Option.iter (mjModel_set_nuser_cam x) mjf_nuser_cam;
  Option.iter (mjModel_set_nuser_tendon x) mjf_nuser_tendon;
  Option.iter (mjModel_set_nuser_actuator x) mjf_nuser_actuator;
  Option.iter (mjModel_set_nuser_sensor x) mjf_nuser_sensor;
  Option.iter (mjModel_set_nnames x) mjf_nnames;
  Option.iter (mjModel_set_nM x) mjf_nM;
  Option.iter (mjModel_set_nemax x) mjf_nemax;
  Option.iter (mjModel_set_njmax x) mjf_njmax;
  Option.iter (mjModel_set_nconmax x) mjf_nconmax;
  Option.iter (mjModel_set_nstack x) mjf_nstack;
  Option.iter (mjModel_set_nuserdata x) mjf_nuserdata;
  Option.iter (mjModel_set_nsensordata x) mjf_nsensordata;
  Option.iter (mjModel_set_nbuffer x) mjf_nbuffer;
  Option.iter (mjModel_set_opt x) mjf_opt;
  Option.iter (mjModel_set_vis x) mjf_vis;
  Option.iter (mjModel_set_stat x) mjf_stat;
  Option.iter (mjModel_set_buffer x) mjf_buffer;
  Option.iter (mjModel_set_qpos0 x) mjf_qpos0;
  Option.iter (mjModel_set_qpos_spring x) mjf_qpos_spring;
  Option.iter (mjModel_set_body_parentid x) mjf_body_parentid;
  Option.iter (mjModel_set_body_rootid x) mjf_body_rootid;
  Option.iter (mjModel_set_body_weldid x) mjf_body_weldid;
  Option.iter (mjModel_set_body_mocapid x) mjf_body_mocapid;
  Option.iter (mjModel_set_body_jntnum x) mjf_body_jntnum;
  Option.iter (mjModel_set_body_jntadr x) mjf_body_jntadr;
  Option.iter (mjModel_set_body_dofnum x) mjf_body_dofnum;
  Option.iter (mjModel_set_body_dofadr x) mjf_body_dofadr;
  Option.iter (mjModel_set_body_geomnum x) mjf_body_geomnum;
  Option.iter (mjModel_set_body_geomadr x) mjf_body_geomadr;
  Option.iter (mjModel_set_body_simple x) mjf_body_simple;
  Option.iter (mjModel_set_body_sameframe x) mjf_body_sameframe;
  Option.iter (mjModel_set_body_pos x) mjf_body_pos;
  Option.iter (mjModel_set_body_quat x) mjf_body_quat;
  Option.iter (mjModel_set_body_ipos x) mjf_body_ipos;
  Option.iter (mjModel_set_body_iquat x) mjf_body_iquat;
  Option.iter (mjModel_set_body_mass x) mjf_body_mass;
  Option.iter (mjModel_set_body_subtreemass x) mjf_body_subtreemass;
  Option.iter (mjModel_set_body_inertia x) mjf_body_inertia;
  Option.iter (mjModel_set_body_invweight0 x) mjf_body_invweight0;
  Option.iter (mjModel_set_body_user x) mjf_body_user;
  Option.iter (mjModel_set_jnt_type x) mjf_jnt_type;
  Option.iter (mjModel_set_jnt_qposadr x) mjf_jnt_qposadr;
  Option.iter (mjModel_set_jnt_dofadr x) mjf_jnt_dofadr;
  Option.iter (mjModel_set_jnt_bodyid x) mjf_jnt_bodyid;
  Option.iter (mjModel_set_jnt_group x) mjf_jnt_group;
  Option.iter (mjModel_set_jnt_limited x) mjf_jnt_limited;
  Option.iter (mjModel_set_jnt_solref x) mjf_jnt_solref;
  Option.iter (mjModel_set_jnt_solimp x) mjf_jnt_solimp;
  Option.iter (mjModel_set_jnt_pos x) mjf_jnt_pos;
  Option.iter (mjModel_set_jnt_axis x) mjf_jnt_axis;
  Option.iter (mjModel_set_jnt_stiffness x) mjf_jnt_stiffness;
  Option.iter (mjModel_set_jnt_range x) mjf_jnt_range;
  Option.iter (mjModel_set_jnt_margin x) mjf_jnt_margin;
  Option.iter (mjModel_set_jnt_user x) mjf_jnt_user;
  Option.iter (mjModel_set_dof_bodyid x) mjf_dof_bodyid;
  Option.iter (mjModel_set_dof_jntid x) mjf_dof_jntid;
  Option.iter (mjModel_set_dof_parentid x) mjf_dof_parentid;
  Option.iter (mjModel_set_dof_Madr x) mjf_dof_Madr;
  Option.iter (mjModel_set_dof_simplenum x) mjf_dof_simplenum;
  Option.iter (mjModel_set_dof_solref x) mjf_dof_solref;
  Option.iter (mjModel_set_dof_solimp x) mjf_dof_solimp;
  Option.iter (mjModel_set_dof_frictionloss x) mjf_dof_frictionloss;
  Option.iter (mjModel_set_dof_armature x) mjf_dof_armature;
  Option.iter (mjModel_set_dof_damping x) mjf_dof_damping;
  Option.iter (mjModel_set_dof_invweight0 x) mjf_dof_invweight0;
  Option.iter (mjModel_set_dof_M0 x) mjf_dof_M0;
  Option.iter (mjModel_set_geom_type x) mjf_geom_type;
  Option.iter (mjModel_set_geom_contype x) mjf_geom_contype;
  Option.iter (mjModel_set_geom_conaffinity x) mjf_geom_conaffinity;
  Option.iter (mjModel_set_geom_condim x) mjf_geom_condim;
  Option.iter (mjModel_set_geom_bodyid x) mjf_geom_bodyid;
  Option.iter (mjModel_set_geom_dataid x) mjf_geom_dataid;
  Option.iter (mjModel_set_geom_matid x) mjf_geom_matid;
  Option.iter (mjModel_set_geom_group x) mjf_geom_group;
  Option.iter (mjModel_set_geom_priority x) mjf_geom_priority;
  Option.iter (mjModel_set_geom_sameframe x) mjf_geom_sameframe;
  Option.iter (mjModel_set_geom_solmix x) mjf_geom_solmix;
  Option.iter (mjModel_set_geom_solref x) mjf_geom_solref;
  Option.iter (mjModel_set_geom_solimp x) mjf_geom_solimp;
  Option.iter (mjModel_set_geom_size x) mjf_geom_size;
  Option.iter (mjModel_set_geom_rbound x) mjf_geom_rbound;
  Option.iter (mjModel_set_geom_pos x) mjf_geom_pos;
  Option.iter (mjModel_set_geom_quat x) mjf_geom_quat;
  Option.iter (mjModel_set_geom_friction x) mjf_geom_friction;
  Option.iter (mjModel_set_geom_margin x) mjf_geom_margin;
  Option.iter (mjModel_set_geom_gap x) mjf_geom_gap;
  Option.iter (mjModel_set_geom_user x) mjf_geom_user;
  Option.iter (mjModel_set_geom_rgba x) mjf_geom_rgba;
  Option.iter (mjModel_set_site_type x) mjf_site_type;
  Option.iter (mjModel_set_site_bodyid x) mjf_site_bodyid;
  Option.iter (mjModel_set_site_matid x) mjf_site_matid;
  Option.iter (mjModel_set_site_group x) mjf_site_group;
  Option.iter (mjModel_set_site_sameframe x) mjf_site_sameframe;
  Option.iter (mjModel_set_site_size x) mjf_site_size;
  Option.iter (mjModel_set_site_pos x) mjf_site_pos;
  Option.iter (mjModel_set_site_quat x) mjf_site_quat;
  Option.iter (mjModel_set_site_user x) mjf_site_user;
  Option.iter (mjModel_set_site_rgba x) mjf_site_rgba;
  Option.iter (mjModel_set_cam_mode x) mjf_cam_mode;
  Option.iter (mjModel_set_cam_bodyid x) mjf_cam_bodyid;
  Option.iter (mjModel_set_cam_targetbodyid x) mjf_cam_targetbodyid;
  Option.iter (mjModel_set_cam_pos x) mjf_cam_pos;
  Option.iter (mjModel_set_cam_quat x) mjf_cam_quat;
  Option.iter (mjModel_set_cam_poscom0 x) mjf_cam_poscom0;
  Option.iter (mjModel_set_cam_pos0 x) mjf_cam_pos0;
  Option.iter (mjModel_set_cam_mat0 x) mjf_cam_mat0;
  Option.iter (mjModel_set_cam_fovy x) mjf_cam_fovy;
  Option.iter (mjModel_set_cam_ipd x) mjf_cam_ipd;
  Option.iter (mjModel_set_cam_user x) mjf_cam_user;
  Option.iter (mjModel_set_light_mode x) mjf_light_mode;
  Option.iter (mjModel_set_light_bodyid x) mjf_light_bodyid;
  Option.iter (mjModel_set_light_targetbodyid x) mjf_light_targetbodyid;
  Option.iter (mjModel_set_light_directional x) mjf_light_directional;
  Option.iter (mjModel_set_light_castshadow x) mjf_light_castshadow;
  Option.iter (mjModel_set_light_active x) mjf_light_active;
  Option.iter (mjModel_set_light_pos x) mjf_light_pos;
  Option.iter (mjModel_set_light_dir x) mjf_light_dir;
  Option.iter (mjModel_set_light_poscom0 x) mjf_light_poscom0;
  Option.iter (mjModel_set_light_pos0 x) mjf_light_pos0;
  Option.iter (mjModel_set_light_dir0 x) mjf_light_dir0;
  Option.iter (mjModel_set_light_attenuation x) mjf_light_attenuation;
  Option.iter (mjModel_set_light_cutoff x) mjf_light_cutoff;
  Option.iter (mjModel_set_light_exponent x) mjf_light_exponent;
  Option.iter (mjModel_set_light_ambient x) mjf_light_ambient;
  Option.iter (mjModel_set_light_diffuse x) mjf_light_diffuse;
  Option.iter (mjModel_set_light_specular x) mjf_light_specular;
  Option.iter (mjModel_set_mesh_vertadr x) mjf_mesh_vertadr;
  Option.iter (mjModel_set_mesh_vertnum x) mjf_mesh_vertnum;
  Option.iter (mjModel_set_mesh_texcoordadr x) mjf_mesh_texcoordadr;
  Option.iter (mjModel_set_mesh_faceadr x) mjf_mesh_faceadr;
  Option.iter (mjModel_set_mesh_facenum x) mjf_mesh_facenum;
  Option.iter (mjModel_set_mesh_graphadr x) mjf_mesh_graphadr;
  Option.iter (mjModel_set_mesh_vert x) mjf_mesh_vert;
  Option.iter (mjModel_set_mesh_normal x) mjf_mesh_normal;
  Option.iter (mjModel_set_mesh_texcoord x) mjf_mesh_texcoord;
  Option.iter (mjModel_set_mesh_face x) mjf_mesh_face;
  Option.iter (mjModel_set_mesh_graph x) mjf_mesh_graph;
  Option.iter (mjModel_set_skin_matid x) mjf_skin_matid;
  Option.iter (mjModel_set_skin_rgba x) mjf_skin_rgba;
  Option.iter (mjModel_set_skin_inflate x) mjf_skin_inflate;
  Option.iter (mjModel_set_skin_vertadr x) mjf_skin_vertadr;
  Option.iter (mjModel_set_skin_vertnum x) mjf_skin_vertnum;
  Option.iter (mjModel_set_skin_texcoordadr x) mjf_skin_texcoordadr;
  Option.iter (mjModel_set_skin_faceadr x) mjf_skin_faceadr;
  Option.iter (mjModel_set_skin_facenum x) mjf_skin_facenum;
  Option.iter (mjModel_set_skin_boneadr x) mjf_skin_boneadr;
  Option.iter (mjModel_set_skin_bonenum x) mjf_skin_bonenum;
  Option.iter (mjModel_set_skin_vert x) mjf_skin_vert;
  Option.iter (mjModel_set_skin_texcoord x) mjf_skin_texcoord;
  Option.iter (mjModel_set_skin_face x) mjf_skin_face;
  Option.iter (mjModel_set_skin_bonevertadr x) mjf_skin_bonevertadr;
  Option.iter (mjModel_set_skin_bonevertnum x) mjf_skin_bonevertnum;
  Option.iter (mjModel_set_skin_bonebindpos x) mjf_skin_bonebindpos;
  Option.iter (mjModel_set_skin_bonebindquat x) mjf_skin_bonebindquat;
  Option.iter (mjModel_set_skin_bonebodyid x) mjf_skin_bonebodyid;
  Option.iter (mjModel_set_skin_bonevertid x) mjf_skin_bonevertid;
  Option.iter (mjModel_set_skin_bonevertweight x) mjf_skin_bonevertweight;
  Option.iter (mjModel_set_hfield_size x) mjf_hfield_size;
  Option.iter (mjModel_set_hfield_nrow x) mjf_hfield_nrow;
  Option.iter (mjModel_set_hfield_ncol x) mjf_hfield_ncol;
  Option.iter (mjModel_set_hfield_adr x) mjf_hfield_adr;
  Option.iter (mjModel_set_hfield_data x) mjf_hfield_data;
  Option.iter (mjModel_set_tex_type x) mjf_tex_type;
  Option.iter (mjModel_set_tex_height x) mjf_tex_height;
  Option.iter (mjModel_set_tex_width x) mjf_tex_width;
  Option.iter (mjModel_set_tex_adr x) mjf_tex_adr;
  Option.iter (mjModel_set_tex_rgb x) mjf_tex_rgb;
  Option.iter (mjModel_set_mat_texid x) mjf_mat_texid;
  Option.iter (mjModel_set_mat_texuniform x) mjf_mat_texuniform;
  Option.iter (mjModel_set_mat_texrepeat x) mjf_mat_texrepeat;
  Option.iter (mjModel_set_mat_emission x) mjf_mat_emission;
  Option.iter (mjModel_set_mat_specular x) mjf_mat_specular;
  Option.iter (mjModel_set_mat_shininess x) mjf_mat_shininess;
  Option.iter (mjModel_set_mat_reflectance x) mjf_mat_reflectance;
  Option.iter (mjModel_set_mat_rgba x) mjf_mat_rgba;
  Option.iter (mjModel_set_pair_dim x) mjf_pair_dim;
  Option.iter (mjModel_set_pair_geom1 x) mjf_pair_geom1;
  Option.iter (mjModel_set_pair_geom2 x) mjf_pair_geom2;
  Option.iter (mjModel_set_pair_signature x) mjf_pair_signature;
  Option.iter (mjModel_set_pair_solref x) mjf_pair_solref;
  Option.iter (mjModel_set_pair_solimp x) mjf_pair_solimp;
  Option.iter (mjModel_set_pair_margin x) mjf_pair_margin;
  Option.iter (mjModel_set_pair_gap x) mjf_pair_gap;
  Option.iter (mjModel_set_pair_friction x) mjf_pair_friction;
  Option.iter (mjModel_set_exclude_signature x) mjf_exclude_signature;
  Option.iter (mjModel_set_eq_type x) mjf_eq_type;
  Option.iter (mjModel_set_eq_obj1id x) mjf_eq_obj1id;
  Option.iter (mjModel_set_eq_obj2id x) mjf_eq_obj2id;
  Option.iter (mjModel_set_eq_active x) mjf_eq_active;
  Option.iter (mjModel_set_eq_solref x) mjf_eq_solref;
  Option.iter (mjModel_set_eq_solimp x) mjf_eq_solimp;
  Option.iter (mjModel_set_eq_data x) mjf_eq_data;
  Option.iter (mjModel_set_tendon_adr x) mjf_tendon_adr;
  Option.iter (mjModel_set_tendon_num x) mjf_tendon_num;
  Option.iter (mjModel_set_tendon_matid x) mjf_tendon_matid;
  Option.iter (mjModel_set_tendon_group x) mjf_tendon_group;
  Option.iter (mjModel_set_tendon_limited x) mjf_tendon_limited;
  Option.iter (mjModel_set_tendon_width x) mjf_tendon_width;
  Option.iter (mjModel_set_tendon_solref_lim x) mjf_tendon_solref_lim;
  Option.iter (mjModel_set_tendon_solimp_lim x) mjf_tendon_solimp_lim;
  Option.iter (mjModel_set_tendon_solref_fri x) mjf_tendon_solref_fri;
  Option.iter (mjModel_set_tendon_solimp_fri x) mjf_tendon_solimp_fri;
  Option.iter (mjModel_set_tendon_range x) mjf_tendon_range;
  Option.iter (mjModel_set_tendon_margin x) mjf_tendon_margin;
  Option.iter (mjModel_set_tendon_stiffness x) mjf_tendon_stiffness;
  Option.iter (mjModel_set_tendon_damping x) mjf_tendon_damping;
  Option.iter (mjModel_set_tendon_frictionloss x) mjf_tendon_frictionloss;
  Option.iter (mjModel_set_tendon_lengthspring x) mjf_tendon_lengthspring;
  Option.iter (mjModel_set_tendon_length0 x) mjf_tendon_length0;
  Option.iter (mjModel_set_tendon_invweight0 x) mjf_tendon_invweight0;
  Option.iter (mjModel_set_tendon_user x) mjf_tendon_user;
  Option.iter (mjModel_set_tendon_rgba x) mjf_tendon_rgba;
  Option.iter (mjModel_set_wrap_type x) mjf_wrap_type;
  Option.iter (mjModel_set_wrap_objid x) mjf_wrap_objid;
  Option.iter (mjModel_set_wrap_prm x) mjf_wrap_prm;
  Option.iter (mjModel_set_actuator_trntype x) mjf_actuator_trntype;
  Option.iter (mjModel_set_actuator_dyntype x) mjf_actuator_dyntype;
  Option.iter (mjModel_set_actuator_gaintype x) mjf_actuator_gaintype;
  Option.iter (mjModel_set_actuator_biastype x) mjf_actuator_biastype;
  Option.iter (mjModel_set_actuator_trnid x) mjf_actuator_trnid;
  Option.iter (mjModel_set_actuator_group x) mjf_actuator_group;
  Option.iter (mjModel_set_actuator_ctrllimited x) mjf_actuator_ctrllimited;
  Option.iter (mjModel_set_actuator_forcelimited x) mjf_actuator_forcelimited;
  Option.iter (mjModel_set_actuator_dynprm x) mjf_actuator_dynprm;
  Option.iter (mjModel_set_actuator_gainprm x) mjf_actuator_gainprm;
  Option.iter (mjModel_set_actuator_biasprm x) mjf_actuator_biasprm;
  Option.iter (mjModel_set_actuator_ctrlrange x) mjf_actuator_ctrlrange;
  Option.iter (mjModel_set_actuator_forcerange x) mjf_actuator_forcerange;
  Option.iter (mjModel_set_actuator_gear x) mjf_actuator_gear;
  Option.iter (mjModel_set_actuator_cranklength x) mjf_actuator_cranklength;
  Option.iter (mjModel_set_actuator_acc0 x) mjf_actuator_acc0;
  Option.iter (mjModel_set_actuator_length0 x) mjf_actuator_length0;
  Option.iter (mjModel_set_actuator_lengthrange x) mjf_actuator_lengthrange;
  Option.iter (mjModel_set_actuator_user x) mjf_actuator_user;
  Option.iter (mjModel_set_sensor_type x) mjf_sensor_type;
  Option.iter (mjModel_set_sensor_datatype x) mjf_sensor_datatype;
  Option.iter (mjModel_set_sensor_needstage x) mjf_sensor_needstage;
  Option.iter (mjModel_set_sensor_objtype x) mjf_sensor_objtype;
  Option.iter (mjModel_set_sensor_objid x) mjf_sensor_objid;
  Option.iter (mjModel_set_sensor_dim x) mjf_sensor_dim;
  Option.iter (mjModel_set_sensor_adr x) mjf_sensor_adr;
  Option.iter (mjModel_set_sensor_cutoff x) mjf_sensor_cutoff;
  Option.iter (mjModel_set_sensor_noise x) mjf_sensor_noise;
  Option.iter (mjModel_set_sensor_user x) mjf_sensor_user;
  Option.iter (mjModel_set_numeric_adr x) mjf_numeric_adr;
  Option.iter (mjModel_set_numeric_size x) mjf_numeric_size;
  Option.iter (mjModel_set_numeric_data x) mjf_numeric_data;
  Option.iter (mjModel_set_text_adr x) mjf_text_adr;
  Option.iter (mjModel_set_text_size x) mjf_text_size;
  Option.iter (mjModel_set_text_data x) mjf_text_data;
  Option.iter (mjModel_set_tuple_adr x) mjf_tuple_adr;
  Option.iter (mjModel_set_tuple_size x) mjf_tuple_size;
  Option.iter (mjModel_set_tuple_objtype x) mjf_tuple_objtype;
  Option.iter (mjModel_set_tuple_objid x) mjf_tuple_objid;
  Option.iter (mjModel_set_tuple_objprm x) mjf_tuple_objprm;
  Option.iter (mjModel_set_key_time x) mjf_key_time;
  Option.iter (mjModel_set_key_qpos x) mjf_key_qpos;
  Option.iter (mjModel_set_key_qvel x) mjf_key_qvel;
  Option.iter (mjModel_set_key_act x) mjf_key_act;
  Option.iter (mjModel_set_key_mpos x) mjf_key_mpos;
  Option.iter (mjModel_set_key_mquat x) mjf_key_mquat;
  Option.iter (mjModel_set_name_bodyadr x) mjf_name_bodyadr;
  Option.iter (mjModel_set_name_jntadr x) mjf_name_jntadr;
  Option.iter (mjModel_set_name_geomadr x) mjf_name_geomadr;
  Option.iter (mjModel_set_name_siteadr x) mjf_name_siteadr;
  Option.iter (mjModel_set_name_camadr x) mjf_name_camadr;
  Option.iter (mjModel_set_name_lightadr x) mjf_name_lightadr;
  Option.iter (mjModel_set_name_meshadr x) mjf_name_meshadr;
  Option.iter (mjModel_set_name_skinadr x) mjf_name_skinadr;
  Option.iter (mjModel_set_name_hfieldadr x) mjf_name_hfieldadr;
  Option.iter (mjModel_set_name_texadr x) mjf_name_texadr;
  Option.iter (mjModel_set_name_matadr x) mjf_name_matadr;
  Option.iter (mjModel_set_name_pairadr x) mjf_name_pairadr;
  Option.iter (mjModel_set_name_excludeadr x) mjf_name_excludeadr;
  Option.iter (mjModel_set_name_eqadr x) mjf_name_eqadr;
  Option.iter (mjModel_set_name_tendonadr x) mjf_name_tendonadr;
  Option.iter (mjModel_set_name_actuatoradr x) mjf_name_actuatoradr;
  Option.iter (mjModel_set_name_sensoradr x) mjf_name_sensoradr;
  Option.iter (mjModel_set_name_numericadr x) mjf_name_numericadr;
  Option.iter (mjModel_set_name_textadr x) mjf_name_textadr;
  Option.iter (mjModel_set_name_tupleadr x) mjf_name_tupleadr;
  Option.iter (mjModel_set_name_keyadr x) mjf_name_keyadr;
  Option.iter (mjModel_set_names x) mjf_names;
  x


(** returns mjModel carray from list of mjModel *)
let mjModel_to_carray = Ctypes.CArray.of_list Typs.mjModel

(** convert mjtWarning type to int *)
let mjtWarning_to_int = function
  | MjWARN_INERTIA     -> Typs.mjWARN_INERTIA |> Int64.to_int
  | MjWARN_CONTACTFULL -> Typs.mjWARN_CONTACTFULL |> Int64.to_int
  | MjWARN_CNSTRFULL   -> Typs.mjWARN_CNSTRFULL |> Int64.to_int
  | MjWARN_VGEOMFULL   -> Typs.mjWARN_VGEOMFULL |> Int64.to_int
  | MjWARN_BADQPOS     -> Typs.mjWARN_BADQPOS |> Int64.to_int
  | MjWARN_BADQVEL     -> Typs.mjWARN_BADQVEL |> Int64.to_int
  | MjWARN_BADQACC     -> Typs.mjWARN_BADQACC |> Int64.to_int
  | MjWARN_BADCTRL     -> Typs.mjWARN_BADCTRL |> Int64.to_int
  | MjNWARNING         -> Typs.mjNWARNING |> Int64.to_int


(** convert mjtTimer type to int *)
let mjtTimer_to_int = function
  | MjTIMER_STEP           -> Typs.mjTIMER_STEP |> Int64.to_int
  | MjTIMER_FORWARD        -> Typs.mjTIMER_FORWARD |> Int64.to_int
  | MjTIMER_INVERSE        -> Typs.mjTIMER_INVERSE |> Int64.to_int
  | MjTIMER_POSITION       -> Typs.mjTIMER_POSITION |> Int64.to_int
  | MjTIMER_VELOCITY       -> Typs.mjTIMER_VELOCITY |> Int64.to_int
  | MjTIMER_ACTUATION      -> Typs.mjTIMER_ACTUATION |> Int64.to_int
  | MjTIMER_ACCELERATION   -> Typs.mjTIMER_ACCELERATION |> Int64.to_int
  | MjTIMER_CONSTRAINT     -> Typs.mjTIMER_CONSTRAINT |> Int64.to_int
  | MjTIMER_POS_KINEMATICS -> Typs.mjTIMER_POS_KINEMATICS |> Int64.to_int
  | MjTIMER_POS_INERTIA    -> Typs.mjTIMER_POS_INERTIA |> Int64.to_int
  | MjTIMER_POS_COLLISION  -> Typs.mjTIMER_POS_COLLISION |> Int64.to_int
  | MjTIMER_POS_MAKE       -> Typs.mjTIMER_POS_MAKE |> Int64.to_int
  | MjTIMER_POS_PROJECT    -> Typs.mjTIMER_POS_PROJECT |> Int64.to_int
  | MjNTIMER               -> Typs.mjNTIMER |> Int64.to_int


(** allocate fresh mjContact struct *)
let mjContact_allocate () = Ctypes.(make Typs.mjContact)

(** make null mjContact struct ptr *)
let mjContact_null () = Ctypes.(from_voidp Typs.mjContact null)

(** get dist from mjContact *)
let mjContact_get_dist x = Ctypes.(getf x Typs.mjContact_dist)

(** set dist for mjContact *)
let mjContact_set_dist x y = Ctypes.(setf x Typs.mjContact_dist y)

(** get pos from mjContact *)
let mjContact_get_pos x = Ctypes.(getf x Typs.mjContact_pos)

(** set pos for mjContact *)
let mjContact_set_pos x y = Ctypes.(setf x Typs.mjContact_pos y)

(** get frame from mjContact *)
let mjContact_get_frame x = Ctypes.(getf x Typs.mjContact_frame)

(** set frame for mjContact *)
let mjContact_set_frame x y = Ctypes.(setf x Typs.mjContact_frame y)

(** get includemargin from mjContact *)
let mjContact_get_includemargin x = Ctypes.(getf x Typs.mjContact_includemargin)

(** set includemargin for mjContact *)
let mjContact_set_includemargin x y = Ctypes.(setf x Typs.mjContact_includemargin y)

(** get friction from mjContact *)
let mjContact_get_friction x = Ctypes.(getf x Typs.mjContact_friction)

(** set friction for mjContact *)
let mjContact_set_friction x y = Ctypes.(setf x Typs.mjContact_friction y)

(** get solref from mjContact *)
let mjContact_get_solref x = Ctypes.(getf x Typs.mjContact_solref)

(** set solref for mjContact *)
let mjContact_set_solref x y = Ctypes.(setf x Typs.mjContact_solref y)

(** get solimp from mjContact *)
let mjContact_get_solimp x = Ctypes.(getf x Typs.mjContact_solimp)

(** set solimp for mjContact *)
let mjContact_set_solimp x y = Ctypes.(setf x Typs.mjContact_solimp y)

(** get mu from mjContact *)
let mjContact_get_mu x = Ctypes.(getf x Typs.mjContact_mu)

(** set mu for mjContact *)
let mjContact_set_mu x y = Ctypes.(setf x Typs.mjContact_mu y)

(** get H from mjContact *)
let mjContact_get_H x = Ctypes.(getf x Typs.mjContact_H)

(** set H for mjContact *)
let mjContact_set_H x y = Ctypes.(setf x Typs.mjContact_H y)

(** get dim from mjContact *)
let mjContact_get_dim x = Ctypes.(getf x Typs.mjContact_dim)

(** set dim for mjContact *)
let mjContact_set_dim x y = Ctypes.(setf x Typs.mjContact_dim y)

(** get geom1 from mjContact *)
let mjContact_get_geom1 x = Ctypes.(getf x Typs.mjContact_geom1)

(** set geom1 for mjContact *)
let mjContact_set_geom1 x y = Ctypes.(setf x Typs.mjContact_geom1 y)

(** get geom2 from mjContact *)
let mjContact_get_geom2 x = Ctypes.(getf x Typs.mjContact_geom2)

(** set geom2 for mjContact *)
let mjContact_set_geom2 x y = Ctypes.(setf x Typs.mjContact_geom2 y)

(** get exclude from mjContact *)
let mjContact_get_exclude x = Ctypes.(getf x Typs.mjContact_exclude)

(** set exclude for mjContact *)
let mjContact_set_exclude x y = Ctypes.(setf x Typs.mjContact_exclude y)

(** get efc_address from mjContact *)
let mjContact_get_efc_address x = Ctypes.(getf x Typs.mjContact_efc_address)

(** set efc_address for mjContact *)
let mjContact_set_efc_address x y = Ctypes.(setf x Typs.mjContact_efc_address y)

(** make mjContact struct *)
let mjContact_make
    ?mjf_dist
    ?mjf_pos
    ?mjf_frame
    ?mjf_includemargin
    ?mjf_friction
    ?mjf_solref
    ?mjf_solimp
    ?mjf_mu
    ?mjf_H
    ?mjf_dim
    ?mjf_geom1
    ?mjf_geom2
    ?mjf_exclude
    ?mjf_efc_address
    ()
  =
  let x = mjContact_allocate () in
  Option.iter (mjContact_set_dist x) mjf_dist;
  Option.iter (mjContact_set_pos x) mjf_pos;
  Option.iter (mjContact_set_frame x) mjf_frame;
  Option.iter (mjContact_set_includemargin x) mjf_includemargin;
  Option.iter (mjContact_set_friction x) mjf_friction;
  Option.iter (mjContact_set_solref x) mjf_solref;
  Option.iter (mjContact_set_solimp x) mjf_solimp;
  Option.iter (mjContact_set_mu x) mjf_mu;
  Option.iter (mjContact_set_H x) mjf_H;
  Option.iter (mjContact_set_dim x) mjf_dim;
  Option.iter (mjContact_set_geom1 x) mjf_geom1;
  Option.iter (mjContact_set_geom2 x) mjf_geom2;
  Option.iter (mjContact_set_exclude x) mjf_exclude;
  Option.iter (mjContact_set_efc_address x) mjf_efc_address;
  x


(** returns mjContact carray from list of mjContact *)
let mjContact_to_carray = Ctypes.CArray.of_list Typs.mjContact

(** allocate fresh mjWarningStat struct *)
let mjWarningStat_allocate () = Ctypes.(make Typs.mjWarningStat)

(** make null mjWarningStat struct ptr *)
let mjWarningStat_null () = Ctypes.(from_voidp Typs.mjWarningStat null)

(** get lastinfo from mjWarningStat *)
let mjWarningStat_get_lastinfo x = Ctypes.(getf x Typs.mjWarningStat_lastinfo)

(** set lastinfo for mjWarningStat *)
let mjWarningStat_set_lastinfo x y = Ctypes.(setf x Typs.mjWarningStat_lastinfo y)

(** get number from mjWarningStat *)
let mjWarningStat_get_number x = Ctypes.(getf x Typs.mjWarningStat_number)

(** set number for mjWarningStat *)
let mjWarningStat_set_number x y = Ctypes.(setf x Typs.mjWarningStat_number y)

(** make mjWarningStat struct *)
let mjWarningStat_make ?mjf_lastinfo ?mjf_number () =
  let x = mjWarningStat_allocate () in
  Option.iter (mjWarningStat_set_lastinfo x) mjf_lastinfo;
  Option.iter (mjWarningStat_set_number x) mjf_number;
  x


(** returns mjWarningStat carray from list of mjWarningStat *)
let mjWarningStat_to_carray = Ctypes.CArray.of_list Typs.mjWarningStat

(** allocate fresh mjTimerStat struct *)
let mjTimerStat_allocate () = Ctypes.(make Typs.mjTimerStat)

(** make null mjTimerStat struct ptr *)
let mjTimerStat_null () = Ctypes.(from_voidp Typs.mjTimerStat null)

(** get duration from mjTimerStat *)
let mjTimerStat_get_duration x = Ctypes.(getf x Typs.mjTimerStat_duration)

(** set duration for mjTimerStat *)
let mjTimerStat_set_duration x y = Ctypes.(setf x Typs.mjTimerStat_duration y)

(** get number from mjTimerStat *)
let mjTimerStat_get_number x = Ctypes.(getf x Typs.mjTimerStat_number)

(** set number for mjTimerStat *)
let mjTimerStat_set_number x y = Ctypes.(setf x Typs.mjTimerStat_number y)

(** make mjTimerStat struct *)
let mjTimerStat_make ?mjf_duration ?mjf_number () =
  let x = mjTimerStat_allocate () in
  Option.iter (mjTimerStat_set_duration x) mjf_duration;
  Option.iter (mjTimerStat_set_number x) mjf_number;
  x


(** returns mjTimerStat carray from list of mjTimerStat *)
let mjTimerStat_to_carray = Ctypes.CArray.of_list Typs.mjTimerStat

(** allocate fresh mjSolverStat struct *)
let mjSolverStat_allocate () = Ctypes.(make Typs.mjSolverStat)

(** make null mjSolverStat struct ptr *)
let mjSolverStat_null () = Ctypes.(from_voidp Typs.mjSolverStat null)

(** get improvement from mjSolverStat *)
let mjSolverStat_get_improvement x = Ctypes.(getf x Typs.mjSolverStat_improvement)

(** set improvement for mjSolverStat *)
let mjSolverStat_set_improvement x y = Ctypes.(setf x Typs.mjSolverStat_improvement y)

(** get gradient from mjSolverStat *)
let mjSolverStat_get_gradient x = Ctypes.(getf x Typs.mjSolverStat_gradient)

(** set gradient for mjSolverStat *)
let mjSolverStat_set_gradient x y = Ctypes.(setf x Typs.mjSolverStat_gradient y)

(** get lineslope from mjSolverStat *)
let mjSolverStat_get_lineslope x = Ctypes.(getf x Typs.mjSolverStat_lineslope)

(** set lineslope for mjSolverStat *)
let mjSolverStat_set_lineslope x y = Ctypes.(setf x Typs.mjSolverStat_lineslope y)

(** get nactive from mjSolverStat *)
let mjSolverStat_get_nactive x = Ctypes.(getf x Typs.mjSolverStat_nactive)

(** set nactive for mjSolverStat *)
let mjSolverStat_set_nactive x y = Ctypes.(setf x Typs.mjSolverStat_nactive y)

(** get nchange from mjSolverStat *)
let mjSolverStat_get_nchange x = Ctypes.(getf x Typs.mjSolverStat_nchange)

(** set nchange for mjSolverStat *)
let mjSolverStat_set_nchange x y = Ctypes.(setf x Typs.mjSolverStat_nchange y)

(** get neval from mjSolverStat *)
let mjSolverStat_get_neval x = Ctypes.(getf x Typs.mjSolverStat_neval)

(** set neval for mjSolverStat *)
let mjSolverStat_set_neval x y = Ctypes.(setf x Typs.mjSolverStat_neval y)

(** get nupdate from mjSolverStat *)
let mjSolverStat_get_nupdate x = Ctypes.(getf x Typs.mjSolverStat_nupdate)

(** set nupdate for mjSolverStat *)
let mjSolverStat_set_nupdate x y = Ctypes.(setf x Typs.mjSolverStat_nupdate y)

(** make mjSolverStat struct *)
let mjSolverStat_make
    ?mjf_improvement
    ?mjf_gradient
    ?mjf_lineslope
    ?mjf_nactive
    ?mjf_nchange
    ?mjf_neval
    ?mjf_nupdate
    ()
  =
  let x = mjSolverStat_allocate () in
  Option.iter (mjSolverStat_set_improvement x) mjf_improvement;
  Option.iter (mjSolverStat_set_gradient x) mjf_gradient;
  Option.iter (mjSolverStat_set_lineslope x) mjf_lineslope;
  Option.iter (mjSolverStat_set_nactive x) mjf_nactive;
  Option.iter (mjSolverStat_set_nchange x) mjf_nchange;
  Option.iter (mjSolverStat_set_neval x) mjf_neval;
  Option.iter (mjSolverStat_set_nupdate x) mjf_nupdate;
  x


(** returns mjSolverStat carray from list of mjSolverStat *)
let mjSolverStat_to_carray = Ctypes.CArray.of_list Typs.mjSolverStat

(** allocate fresh mjData struct *)
let mjData_allocate () = Ctypes.(make Typs.mjData)

(** make null mjData struct ptr *)
let mjData_null () = Ctypes.(from_voidp Typs.mjData null)

(** get nstack from mjData *)
let mjData_get_nstack x = Ctypes.(getf x Typs.mjData_nstack)

(** set nstack for mjData *)
let mjData_set_nstack x y = Ctypes.(setf x Typs.mjData_nstack y)

(** get nbuffer from mjData *)
let mjData_get_nbuffer x = Ctypes.(getf x Typs.mjData_nbuffer)

(** set nbuffer for mjData *)
let mjData_set_nbuffer x y = Ctypes.(setf x Typs.mjData_nbuffer y)

(** get pstack from mjData *)
let mjData_get_pstack x = Ctypes.(getf x Typs.mjData_pstack)

(** set pstack for mjData *)
let mjData_set_pstack x y = Ctypes.(setf x Typs.mjData_pstack y)

(** get maxuse_stack from mjData *)
let mjData_get_maxuse_stack x = Ctypes.(getf x Typs.mjData_maxuse_stack)

(** set maxuse_stack for mjData *)
let mjData_set_maxuse_stack x y = Ctypes.(setf x Typs.mjData_maxuse_stack y)

(** get maxuse_con from mjData *)
let mjData_get_maxuse_con x = Ctypes.(getf x Typs.mjData_maxuse_con)

(** set maxuse_con for mjData *)
let mjData_set_maxuse_con x y = Ctypes.(setf x Typs.mjData_maxuse_con y)

(** get maxuse_efc from mjData *)
let mjData_get_maxuse_efc x = Ctypes.(getf x Typs.mjData_maxuse_efc)

(** set maxuse_efc for mjData *)
let mjData_set_maxuse_efc x y = Ctypes.(setf x Typs.mjData_maxuse_efc y)

(** get warning from mjData *)
let mjData_get_warning x = Ctypes.(getf x Typs.mjData_warning)

(** set warning for mjData *)
let mjData_set_warning x y = Ctypes.(setf x Typs.mjData_warning y)

(** get timer from mjData *)
let mjData_get_timer x = Ctypes.(getf x Typs.mjData_timer)

(** set timer for mjData *)
let mjData_set_timer x y = Ctypes.(setf x Typs.mjData_timer y)

(** get solver from mjData *)
let mjData_get_solver x = Ctypes.(getf x Typs.mjData_solver)

(** set solver for mjData *)
let mjData_set_solver x y = Ctypes.(setf x Typs.mjData_solver y)

(** get solver_iter from mjData *)
let mjData_get_solver_iter x = Ctypes.(getf x Typs.mjData_solver_iter)

(** set solver_iter for mjData *)
let mjData_set_solver_iter x y = Ctypes.(setf x Typs.mjData_solver_iter y)

(** get solver_nnz from mjData *)
let mjData_get_solver_nnz x = Ctypes.(getf x Typs.mjData_solver_nnz)

(** set solver_nnz for mjData *)
let mjData_set_solver_nnz x y = Ctypes.(setf x Typs.mjData_solver_nnz y)

(** get solver_fwdinv from mjData *)
let mjData_get_solver_fwdinv x = Ctypes.(getf x Typs.mjData_solver_fwdinv)

(** set solver_fwdinv for mjData *)
let mjData_set_solver_fwdinv x y = Ctypes.(setf x Typs.mjData_solver_fwdinv y)

(** get ne from mjData *)
let mjData_get_ne x = Ctypes.(getf x Typs.mjData_ne)

(** set ne for mjData *)
let mjData_set_ne x y = Ctypes.(setf x Typs.mjData_ne y)

(** get nf from mjData *)
let mjData_get_nf x = Ctypes.(getf x Typs.mjData_nf)

(** set nf for mjData *)
let mjData_set_nf x y = Ctypes.(setf x Typs.mjData_nf y)

(** get nefc from mjData *)
let mjData_get_nefc x = Ctypes.(getf x Typs.mjData_nefc)

(** set nefc for mjData *)
let mjData_set_nefc x y = Ctypes.(setf x Typs.mjData_nefc y)

(** get ncon from mjData *)
let mjData_get_ncon x = Ctypes.(getf x Typs.mjData_ncon)

(** set ncon for mjData *)
let mjData_set_ncon x y = Ctypes.(setf x Typs.mjData_ncon y)

(** get time from mjData *)
let mjData_get_time x = Ctypes.(getf x Typs.mjData_time)

(** set time for mjData *)
let mjData_set_time x y = Ctypes.(setf x Typs.mjData_time y)

(** get energy from mjData *)
let mjData_get_energy x = Ctypes.(getf x Typs.mjData_energy)

(** set energy for mjData *)
let mjData_set_energy x y = Ctypes.(setf x Typs.mjData_energy y)

(** get buffer from mjData *)
let mjData_get_buffer x = Ctypes.(getf x Typs.mjData_buffer)

(** set buffer for mjData *)
let mjData_set_buffer x y = Ctypes.(setf x Typs.mjData_buffer y)

(** get stack from mjData *)
let mjData_get_stack x = Ctypes.(getf x Typs.mjData_stack)

(** set stack for mjData *)
let mjData_set_stack x y = Ctypes.(setf x Typs.mjData_stack y)

(** get qpos from mjData *)
let mjData_get_qpos x = Ctypes.(getf x Typs.mjData_qpos)

(** set qpos for mjData *)
let mjData_set_qpos x y = Ctypes.(setf x Typs.mjData_qpos y)

(** get qvel from mjData *)
let mjData_get_qvel x = Ctypes.(getf x Typs.mjData_qvel)

(** set qvel for mjData *)
let mjData_set_qvel x y = Ctypes.(setf x Typs.mjData_qvel y)

(** get act from mjData *)
let mjData_get_act x = Ctypes.(getf x Typs.mjData_act)

(** set act for mjData *)
let mjData_set_act x y = Ctypes.(setf x Typs.mjData_act y)

(** get qacc_warmstart from mjData *)
let mjData_get_qacc_warmstart x = Ctypes.(getf x Typs.mjData_qacc_warmstart)

(** set qacc_warmstart for mjData *)
let mjData_set_qacc_warmstart x y = Ctypes.(setf x Typs.mjData_qacc_warmstart y)

(** get ctrl from mjData *)
let mjData_get_ctrl x = Ctypes.(getf x Typs.mjData_ctrl)

(** set ctrl for mjData *)
let mjData_set_ctrl x y = Ctypes.(setf x Typs.mjData_ctrl y)

(** get qfrc_applied from mjData *)
let mjData_get_qfrc_applied x = Ctypes.(getf x Typs.mjData_qfrc_applied)

(** set qfrc_applied for mjData *)
let mjData_set_qfrc_applied x y = Ctypes.(setf x Typs.mjData_qfrc_applied y)

(** get xfrc_applied from mjData *)
let mjData_get_xfrc_applied x = Ctypes.(getf x Typs.mjData_xfrc_applied)

(** set xfrc_applied for mjData *)
let mjData_set_xfrc_applied x y = Ctypes.(setf x Typs.mjData_xfrc_applied y)

(** get qacc from mjData *)
let mjData_get_qacc x = Ctypes.(getf x Typs.mjData_qacc)

(** set qacc for mjData *)
let mjData_set_qacc x y = Ctypes.(setf x Typs.mjData_qacc y)

(** get act_dot from mjData *)
let mjData_get_act_dot x = Ctypes.(getf x Typs.mjData_act_dot)

(** set act_dot for mjData *)
let mjData_set_act_dot x y = Ctypes.(setf x Typs.mjData_act_dot y)

(** get mocap_pos from mjData *)
let mjData_get_mocap_pos x = Ctypes.(getf x Typs.mjData_mocap_pos)

(** set mocap_pos for mjData *)
let mjData_set_mocap_pos x y = Ctypes.(setf x Typs.mjData_mocap_pos y)

(** get mocap_quat from mjData *)
let mjData_get_mocap_quat x = Ctypes.(getf x Typs.mjData_mocap_quat)

(** set mocap_quat for mjData *)
let mjData_set_mocap_quat x y = Ctypes.(setf x Typs.mjData_mocap_quat y)

(** get userdata from mjData *)
let mjData_get_userdata x = Ctypes.(getf x Typs.mjData_userdata)

(** set userdata for mjData *)
let mjData_set_userdata x y = Ctypes.(setf x Typs.mjData_userdata y)

(** get sensordata from mjData *)
let mjData_get_sensordata x = Ctypes.(getf x Typs.mjData_sensordata)

(** set sensordata for mjData *)
let mjData_set_sensordata x y = Ctypes.(setf x Typs.mjData_sensordata y)

(** get xpos from mjData *)
let mjData_get_xpos x = Ctypes.(getf x Typs.mjData_xpos)

(** set xpos for mjData *)
let mjData_set_xpos x y = Ctypes.(setf x Typs.mjData_xpos y)

(** get xquat from mjData *)
let mjData_get_xquat x = Ctypes.(getf x Typs.mjData_xquat)

(** set xquat for mjData *)
let mjData_set_xquat x y = Ctypes.(setf x Typs.mjData_xquat y)

(** get xmat from mjData *)
let mjData_get_xmat x = Ctypes.(getf x Typs.mjData_xmat)

(** set xmat for mjData *)
let mjData_set_xmat x y = Ctypes.(setf x Typs.mjData_xmat y)

(** get xipos from mjData *)
let mjData_get_xipos x = Ctypes.(getf x Typs.mjData_xipos)

(** set xipos for mjData *)
let mjData_set_xipos x y = Ctypes.(setf x Typs.mjData_xipos y)

(** get ximat from mjData *)
let mjData_get_ximat x = Ctypes.(getf x Typs.mjData_ximat)

(** set ximat for mjData *)
let mjData_set_ximat x y = Ctypes.(setf x Typs.mjData_ximat y)

(** get xanchor from mjData *)
let mjData_get_xanchor x = Ctypes.(getf x Typs.mjData_xanchor)

(** set xanchor for mjData *)
let mjData_set_xanchor x y = Ctypes.(setf x Typs.mjData_xanchor y)

(** get xaxis from mjData *)
let mjData_get_xaxis x = Ctypes.(getf x Typs.mjData_xaxis)

(** set xaxis for mjData *)
let mjData_set_xaxis x y = Ctypes.(setf x Typs.mjData_xaxis y)

(** get geom_xpos from mjData *)
let mjData_get_geom_xpos x = Ctypes.(getf x Typs.mjData_geom_xpos)

(** set geom_xpos for mjData *)
let mjData_set_geom_xpos x y = Ctypes.(setf x Typs.mjData_geom_xpos y)

(** get geom_xmat from mjData *)
let mjData_get_geom_xmat x = Ctypes.(getf x Typs.mjData_geom_xmat)

(** set geom_xmat for mjData *)
let mjData_set_geom_xmat x y = Ctypes.(setf x Typs.mjData_geom_xmat y)

(** get site_xpos from mjData *)
let mjData_get_site_xpos x = Ctypes.(getf x Typs.mjData_site_xpos)

(** set site_xpos for mjData *)
let mjData_set_site_xpos x y = Ctypes.(setf x Typs.mjData_site_xpos y)

(** get site_xmat from mjData *)
let mjData_get_site_xmat x = Ctypes.(getf x Typs.mjData_site_xmat)

(** set site_xmat for mjData *)
let mjData_set_site_xmat x y = Ctypes.(setf x Typs.mjData_site_xmat y)

(** get cam_xpos from mjData *)
let mjData_get_cam_xpos x = Ctypes.(getf x Typs.mjData_cam_xpos)

(** set cam_xpos for mjData *)
let mjData_set_cam_xpos x y = Ctypes.(setf x Typs.mjData_cam_xpos y)

(** get cam_xmat from mjData *)
let mjData_get_cam_xmat x = Ctypes.(getf x Typs.mjData_cam_xmat)

(** set cam_xmat for mjData *)
let mjData_set_cam_xmat x y = Ctypes.(setf x Typs.mjData_cam_xmat y)

(** get light_xpos from mjData *)
let mjData_get_light_xpos x = Ctypes.(getf x Typs.mjData_light_xpos)

(** set light_xpos for mjData *)
let mjData_set_light_xpos x y = Ctypes.(setf x Typs.mjData_light_xpos y)

(** get light_xdir from mjData *)
let mjData_get_light_xdir x = Ctypes.(getf x Typs.mjData_light_xdir)

(** set light_xdir for mjData *)
let mjData_set_light_xdir x y = Ctypes.(setf x Typs.mjData_light_xdir y)

(** get subtree_com from mjData *)
let mjData_get_subtree_com x = Ctypes.(getf x Typs.mjData_subtree_com)

(** set subtree_com for mjData *)
let mjData_set_subtree_com x y = Ctypes.(setf x Typs.mjData_subtree_com y)

(** get cdof from mjData *)
let mjData_get_cdof x = Ctypes.(getf x Typs.mjData_cdof)

(** set cdof for mjData *)
let mjData_set_cdof x y = Ctypes.(setf x Typs.mjData_cdof y)

(** get cinert from mjData *)
let mjData_get_cinert x = Ctypes.(getf x Typs.mjData_cinert)

(** set cinert for mjData *)
let mjData_set_cinert x y = Ctypes.(setf x Typs.mjData_cinert y)

(** get ten_wrapadr from mjData *)
let mjData_get_ten_wrapadr x = Ctypes.(getf x Typs.mjData_ten_wrapadr)

(** set ten_wrapadr for mjData *)
let mjData_set_ten_wrapadr x y = Ctypes.(setf x Typs.mjData_ten_wrapadr y)

(** get ten_wrapnum from mjData *)
let mjData_get_ten_wrapnum x = Ctypes.(getf x Typs.mjData_ten_wrapnum)

(** set ten_wrapnum for mjData *)
let mjData_set_ten_wrapnum x y = Ctypes.(setf x Typs.mjData_ten_wrapnum y)

(** get ten_J_rownnz from mjData *)
let mjData_get_ten_J_rownnz x = Ctypes.(getf x Typs.mjData_ten_J_rownnz)

(** set ten_J_rownnz for mjData *)
let mjData_set_ten_J_rownnz x y = Ctypes.(setf x Typs.mjData_ten_J_rownnz y)

(** get ten_J_rowadr from mjData *)
let mjData_get_ten_J_rowadr x = Ctypes.(getf x Typs.mjData_ten_J_rowadr)

(** set ten_J_rowadr for mjData *)
let mjData_set_ten_J_rowadr x y = Ctypes.(setf x Typs.mjData_ten_J_rowadr y)

(** get ten_J_colind from mjData *)
let mjData_get_ten_J_colind x = Ctypes.(getf x Typs.mjData_ten_J_colind)

(** set ten_J_colind for mjData *)
let mjData_set_ten_J_colind x y = Ctypes.(setf x Typs.mjData_ten_J_colind y)

(** get ten_length from mjData *)
let mjData_get_ten_length x = Ctypes.(getf x Typs.mjData_ten_length)

(** set ten_length for mjData *)
let mjData_set_ten_length x y = Ctypes.(setf x Typs.mjData_ten_length y)

(** get ten_J from mjData *)
let mjData_get_ten_J x = Ctypes.(getf x Typs.mjData_ten_J)

(** set ten_J for mjData *)
let mjData_set_ten_J x y = Ctypes.(setf x Typs.mjData_ten_J y)

(** get wrap_obj from mjData *)
let mjData_get_wrap_obj x = Ctypes.(getf x Typs.mjData_wrap_obj)

(** set wrap_obj for mjData *)
let mjData_set_wrap_obj x y = Ctypes.(setf x Typs.mjData_wrap_obj y)

(** get wrap_xpos from mjData *)
let mjData_get_wrap_xpos x = Ctypes.(getf x Typs.mjData_wrap_xpos)

(** set wrap_xpos for mjData *)
let mjData_set_wrap_xpos x y = Ctypes.(setf x Typs.mjData_wrap_xpos y)

(** get actuator_length from mjData *)
let mjData_get_actuator_length x = Ctypes.(getf x Typs.mjData_actuator_length)

(** set actuator_length for mjData *)
let mjData_set_actuator_length x y = Ctypes.(setf x Typs.mjData_actuator_length y)

(** get actuator_moment from mjData *)
let mjData_get_actuator_moment x = Ctypes.(getf x Typs.mjData_actuator_moment)

(** set actuator_moment for mjData *)
let mjData_set_actuator_moment x y = Ctypes.(setf x Typs.mjData_actuator_moment y)

(** get crb from mjData *)
let mjData_get_crb x = Ctypes.(getf x Typs.mjData_crb)

(** set crb for mjData *)
let mjData_set_crb x y = Ctypes.(setf x Typs.mjData_crb y)

(** get qM from mjData *)
let mjData_get_qM x = Ctypes.(getf x Typs.mjData_qM)

(** set qM for mjData *)
let mjData_set_qM x y = Ctypes.(setf x Typs.mjData_qM y)

(** get qLD from mjData *)
let mjData_get_qLD x = Ctypes.(getf x Typs.mjData_qLD)

(** set qLD for mjData *)
let mjData_set_qLD x y = Ctypes.(setf x Typs.mjData_qLD y)

(** get qLDiagInv from mjData *)
let mjData_get_qLDiagInv x = Ctypes.(getf x Typs.mjData_qLDiagInv)

(** set qLDiagInv for mjData *)
let mjData_set_qLDiagInv x y = Ctypes.(setf x Typs.mjData_qLDiagInv y)

(** get qLDiagSqrtInv from mjData *)
let mjData_get_qLDiagSqrtInv x = Ctypes.(getf x Typs.mjData_qLDiagSqrtInv)

(** set qLDiagSqrtInv for mjData *)
let mjData_set_qLDiagSqrtInv x y = Ctypes.(setf x Typs.mjData_qLDiagSqrtInv y)

(** get contact from mjData *)
let mjData_get_contact x = Ctypes.(getf x Typs.mjData_contact)

(** set contact for mjData *)
let mjData_set_contact x y = Ctypes.(setf x Typs.mjData_contact y)

(** get efc_type from mjData *)
let mjData_get_efc_type x = Ctypes.(getf x Typs.mjData_efc_type)

(** set efc_type for mjData *)
let mjData_set_efc_type x y = Ctypes.(setf x Typs.mjData_efc_type y)

(** get efc_id from mjData *)
let mjData_get_efc_id x = Ctypes.(getf x Typs.mjData_efc_id)

(** set efc_id for mjData *)
let mjData_set_efc_id x y = Ctypes.(setf x Typs.mjData_efc_id y)

(** get efc_J_rownnz from mjData *)
let mjData_get_efc_J_rownnz x = Ctypes.(getf x Typs.mjData_efc_J_rownnz)

(** set efc_J_rownnz for mjData *)
let mjData_set_efc_J_rownnz x y = Ctypes.(setf x Typs.mjData_efc_J_rownnz y)

(** get efc_J_rowadr from mjData *)
let mjData_get_efc_J_rowadr x = Ctypes.(getf x Typs.mjData_efc_J_rowadr)

(** set efc_J_rowadr for mjData *)
let mjData_set_efc_J_rowadr x y = Ctypes.(setf x Typs.mjData_efc_J_rowadr y)

(** get efc_J_rowsuper from mjData *)
let mjData_get_efc_J_rowsuper x = Ctypes.(getf x Typs.mjData_efc_J_rowsuper)

(** set efc_J_rowsuper for mjData *)
let mjData_set_efc_J_rowsuper x y = Ctypes.(setf x Typs.mjData_efc_J_rowsuper y)

(** get efc_J_colind from mjData *)
let mjData_get_efc_J_colind x = Ctypes.(getf x Typs.mjData_efc_J_colind)

(** set efc_J_colind for mjData *)
let mjData_set_efc_J_colind x y = Ctypes.(setf x Typs.mjData_efc_J_colind y)

(** get efc_JT_rownnz from mjData *)
let mjData_get_efc_JT_rownnz x = Ctypes.(getf x Typs.mjData_efc_JT_rownnz)

(** set efc_JT_rownnz for mjData *)
let mjData_set_efc_JT_rownnz x y = Ctypes.(setf x Typs.mjData_efc_JT_rownnz y)

(** get efc_JT_rowadr from mjData *)
let mjData_get_efc_JT_rowadr x = Ctypes.(getf x Typs.mjData_efc_JT_rowadr)

(** set efc_JT_rowadr for mjData *)
let mjData_set_efc_JT_rowadr x y = Ctypes.(setf x Typs.mjData_efc_JT_rowadr y)

(** get efc_JT_rowsuper from mjData *)
let mjData_get_efc_JT_rowsuper x = Ctypes.(getf x Typs.mjData_efc_JT_rowsuper)

(** set efc_JT_rowsuper for mjData *)
let mjData_set_efc_JT_rowsuper x y = Ctypes.(setf x Typs.mjData_efc_JT_rowsuper y)

(** get efc_JT_colind from mjData *)
let mjData_get_efc_JT_colind x = Ctypes.(getf x Typs.mjData_efc_JT_colind)

(** set efc_JT_colind for mjData *)
let mjData_set_efc_JT_colind x y = Ctypes.(setf x Typs.mjData_efc_JT_colind y)

(** get efc_J from mjData *)
let mjData_get_efc_J x = Ctypes.(getf x Typs.mjData_efc_J)

(** set efc_J for mjData *)
let mjData_set_efc_J x y = Ctypes.(setf x Typs.mjData_efc_J y)

(** get efc_JT from mjData *)
let mjData_get_efc_JT x = Ctypes.(getf x Typs.mjData_efc_JT)

(** set efc_JT for mjData *)
let mjData_set_efc_JT x y = Ctypes.(setf x Typs.mjData_efc_JT y)

(** get efc_pos from mjData *)
let mjData_get_efc_pos x = Ctypes.(getf x Typs.mjData_efc_pos)

(** set efc_pos for mjData *)
let mjData_set_efc_pos x y = Ctypes.(setf x Typs.mjData_efc_pos y)

(** get efc_margin from mjData *)
let mjData_get_efc_margin x = Ctypes.(getf x Typs.mjData_efc_margin)

(** set efc_margin for mjData *)
let mjData_set_efc_margin x y = Ctypes.(setf x Typs.mjData_efc_margin y)

(** get efc_frictionloss from mjData *)
let mjData_get_efc_frictionloss x = Ctypes.(getf x Typs.mjData_efc_frictionloss)

(** set efc_frictionloss for mjData *)
let mjData_set_efc_frictionloss x y = Ctypes.(setf x Typs.mjData_efc_frictionloss y)

(** get efc_diagApprox from mjData *)
let mjData_get_efc_diagApprox x = Ctypes.(getf x Typs.mjData_efc_diagApprox)

(** set efc_diagApprox for mjData *)
let mjData_set_efc_diagApprox x y = Ctypes.(setf x Typs.mjData_efc_diagApprox y)

(** get efc_KBIP from mjData *)
let mjData_get_efc_KBIP x = Ctypes.(getf x Typs.mjData_efc_KBIP)

(** set efc_KBIP for mjData *)
let mjData_set_efc_KBIP x y = Ctypes.(setf x Typs.mjData_efc_KBIP y)

(** get efc_D from mjData *)
let mjData_get_efc_D x = Ctypes.(getf x Typs.mjData_efc_D)

(** set efc_D for mjData *)
let mjData_set_efc_D x y = Ctypes.(setf x Typs.mjData_efc_D y)

(** get efc_R from mjData *)
let mjData_get_efc_R x = Ctypes.(getf x Typs.mjData_efc_R)

(** set efc_R for mjData *)
let mjData_set_efc_R x y = Ctypes.(setf x Typs.mjData_efc_R y)

(** get efc_AR_rownnz from mjData *)
let mjData_get_efc_AR_rownnz x = Ctypes.(getf x Typs.mjData_efc_AR_rownnz)

(** set efc_AR_rownnz for mjData *)
let mjData_set_efc_AR_rownnz x y = Ctypes.(setf x Typs.mjData_efc_AR_rownnz y)

(** get efc_AR_rowadr from mjData *)
let mjData_get_efc_AR_rowadr x = Ctypes.(getf x Typs.mjData_efc_AR_rowadr)

(** set efc_AR_rowadr for mjData *)
let mjData_set_efc_AR_rowadr x y = Ctypes.(setf x Typs.mjData_efc_AR_rowadr y)

(** get efc_AR_colind from mjData *)
let mjData_get_efc_AR_colind x = Ctypes.(getf x Typs.mjData_efc_AR_colind)

(** set efc_AR_colind for mjData *)
let mjData_set_efc_AR_colind x y = Ctypes.(setf x Typs.mjData_efc_AR_colind y)

(** get efc_AR from mjData *)
let mjData_get_efc_AR x = Ctypes.(getf x Typs.mjData_efc_AR)

(** set efc_AR for mjData *)
let mjData_set_efc_AR x y = Ctypes.(setf x Typs.mjData_efc_AR y)

(** get ten_velocity from mjData *)
let mjData_get_ten_velocity x = Ctypes.(getf x Typs.mjData_ten_velocity)

(** set ten_velocity for mjData *)
let mjData_set_ten_velocity x y = Ctypes.(setf x Typs.mjData_ten_velocity y)

(** get actuator_velocity from mjData *)
let mjData_get_actuator_velocity x = Ctypes.(getf x Typs.mjData_actuator_velocity)

(** set actuator_velocity for mjData *)
let mjData_set_actuator_velocity x y = Ctypes.(setf x Typs.mjData_actuator_velocity y)

(** get cvel from mjData *)
let mjData_get_cvel x = Ctypes.(getf x Typs.mjData_cvel)

(** set cvel for mjData *)
let mjData_set_cvel x y = Ctypes.(setf x Typs.mjData_cvel y)

(** get cdof_dot from mjData *)
let mjData_get_cdof_dot x = Ctypes.(getf x Typs.mjData_cdof_dot)

(** set cdof_dot for mjData *)
let mjData_set_cdof_dot x y = Ctypes.(setf x Typs.mjData_cdof_dot y)

(** get qfrc_bias from mjData *)
let mjData_get_qfrc_bias x = Ctypes.(getf x Typs.mjData_qfrc_bias)

(** set qfrc_bias for mjData *)
let mjData_set_qfrc_bias x y = Ctypes.(setf x Typs.mjData_qfrc_bias y)

(** get qfrc_passive from mjData *)
let mjData_get_qfrc_passive x = Ctypes.(getf x Typs.mjData_qfrc_passive)

(** set qfrc_passive for mjData *)
let mjData_set_qfrc_passive x y = Ctypes.(setf x Typs.mjData_qfrc_passive y)

(** get efc_vel from mjData *)
let mjData_get_efc_vel x = Ctypes.(getf x Typs.mjData_efc_vel)

(** set efc_vel for mjData *)
let mjData_set_efc_vel x y = Ctypes.(setf x Typs.mjData_efc_vel y)

(** get efc_aref from mjData *)
let mjData_get_efc_aref x = Ctypes.(getf x Typs.mjData_efc_aref)

(** set efc_aref for mjData *)
let mjData_set_efc_aref x y = Ctypes.(setf x Typs.mjData_efc_aref y)

(** get subtree_linvel from mjData *)
let mjData_get_subtree_linvel x = Ctypes.(getf x Typs.mjData_subtree_linvel)

(** set subtree_linvel for mjData *)
let mjData_set_subtree_linvel x y = Ctypes.(setf x Typs.mjData_subtree_linvel y)

(** get subtree_angmom from mjData *)
let mjData_get_subtree_angmom x = Ctypes.(getf x Typs.mjData_subtree_angmom)

(** set subtree_angmom for mjData *)
let mjData_set_subtree_angmom x y = Ctypes.(setf x Typs.mjData_subtree_angmom y)

(** get actuator_force from mjData *)
let mjData_get_actuator_force x = Ctypes.(getf x Typs.mjData_actuator_force)

(** set actuator_force for mjData *)
let mjData_set_actuator_force x y = Ctypes.(setf x Typs.mjData_actuator_force y)

(** get qfrc_actuator from mjData *)
let mjData_get_qfrc_actuator x = Ctypes.(getf x Typs.mjData_qfrc_actuator)

(** set qfrc_actuator for mjData *)
let mjData_set_qfrc_actuator x y = Ctypes.(setf x Typs.mjData_qfrc_actuator y)

(** get qfrc_unc from mjData *)
let mjData_get_qfrc_unc x = Ctypes.(getf x Typs.mjData_qfrc_unc)

(** set qfrc_unc for mjData *)
let mjData_set_qfrc_unc x y = Ctypes.(setf x Typs.mjData_qfrc_unc y)

(** get qacc_unc from mjData *)
let mjData_get_qacc_unc x = Ctypes.(getf x Typs.mjData_qacc_unc)

(** set qacc_unc for mjData *)
let mjData_set_qacc_unc x y = Ctypes.(setf x Typs.mjData_qacc_unc y)

(** get efc_b from mjData *)
let mjData_get_efc_b x = Ctypes.(getf x Typs.mjData_efc_b)

(** set efc_b for mjData *)
let mjData_set_efc_b x y = Ctypes.(setf x Typs.mjData_efc_b y)

(** get efc_force from mjData *)
let mjData_get_efc_force x = Ctypes.(getf x Typs.mjData_efc_force)

(** set efc_force for mjData *)
let mjData_set_efc_force x y = Ctypes.(setf x Typs.mjData_efc_force y)

(** get efc_state from mjData *)
let mjData_get_efc_state x = Ctypes.(getf x Typs.mjData_efc_state)

(** set efc_state for mjData *)
let mjData_set_efc_state x y = Ctypes.(setf x Typs.mjData_efc_state y)

(** get qfrc_constraint from mjData *)
let mjData_get_qfrc_constraint x = Ctypes.(getf x Typs.mjData_qfrc_constraint)

(** set qfrc_constraint for mjData *)
let mjData_set_qfrc_constraint x y = Ctypes.(setf x Typs.mjData_qfrc_constraint y)

(** get qfrc_inverse from mjData *)
let mjData_get_qfrc_inverse x = Ctypes.(getf x Typs.mjData_qfrc_inverse)

(** set qfrc_inverse for mjData *)
let mjData_set_qfrc_inverse x y = Ctypes.(setf x Typs.mjData_qfrc_inverse y)

(** get cacc from mjData *)
let mjData_get_cacc x = Ctypes.(getf x Typs.mjData_cacc)

(** set cacc for mjData *)
let mjData_set_cacc x y = Ctypes.(setf x Typs.mjData_cacc y)

(** get cfrc_int from mjData *)
let mjData_get_cfrc_int x = Ctypes.(getf x Typs.mjData_cfrc_int)

(** set cfrc_int for mjData *)
let mjData_set_cfrc_int x y = Ctypes.(setf x Typs.mjData_cfrc_int y)

(** get cfrc_ext from mjData *)
let mjData_get_cfrc_ext x = Ctypes.(getf x Typs.mjData_cfrc_ext)

(** set cfrc_ext for mjData *)
let mjData_set_cfrc_ext x y = Ctypes.(setf x Typs.mjData_cfrc_ext y)

(** make mjData struct *)
let mjData_make
    ?mjf_nstack
    ?mjf_nbuffer
    ?mjf_pstack
    ?mjf_maxuse_stack
    ?mjf_maxuse_con
    ?mjf_maxuse_efc
    ?mjf_warning
    ?mjf_timer
    ?mjf_solver
    ?mjf_solver_iter
    ?mjf_solver_nnz
    ?mjf_solver_fwdinv
    ?mjf_ne
    ?mjf_nf
    ?mjf_nefc
    ?mjf_ncon
    ?mjf_time
    ?mjf_energy
    ?mjf_buffer
    ?mjf_stack
    ?mjf_qpos
    ?mjf_qvel
    ?mjf_act
    ?mjf_qacc_warmstart
    ?mjf_ctrl
    ?mjf_qfrc_applied
    ?mjf_xfrc_applied
    ?mjf_qacc
    ?mjf_act_dot
    ?mjf_mocap_pos
    ?mjf_mocap_quat
    ?mjf_userdata
    ?mjf_sensordata
    ?mjf_xpos
    ?mjf_xquat
    ?mjf_xmat
    ?mjf_xipos
    ?mjf_ximat
    ?mjf_xanchor
    ?mjf_xaxis
    ?mjf_geom_xpos
    ?mjf_geom_xmat
    ?mjf_site_xpos
    ?mjf_site_xmat
    ?mjf_cam_xpos
    ?mjf_cam_xmat
    ?mjf_light_xpos
    ?mjf_light_xdir
    ?mjf_subtree_com
    ?mjf_cdof
    ?mjf_cinert
    ?mjf_ten_wrapadr
    ?mjf_ten_wrapnum
    ?mjf_ten_J_rownnz
    ?mjf_ten_J_rowadr
    ?mjf_ten_J_colind
    ?mjf_ten_length
    ?mjf_ten_J
    ?mjf_wrap_obj
    ?mjf_wrap_xpos
    ?mjf_actuator_length
    ?mjf_actuator_moment
    ?mjf_crb
    ?mjf_qM
    ?mjf_qLD
    ?mjf_qLDiagInv
    ?mjf_qLDiagSqrtInv
    ?mjf_contact
    ?mjf_efc_type
    ?mjf_efc_id
    ?mjf_efc_J_rownnz
    ?mjf_efc_J_rowadr
    ?mjf_efc_J_rowsuper
    ?mjf_efc_J_colind
    ?mjf_efc_JT_rownnz
    ?mjf_efc_JT_rowadr
    ?mjf_efc_JT_rowsuper
    ?mjf_efc_JT_colind
    ?mjf_efc_J
    ?mjf_efc_JT
    ?mjf_efc_pos
    ?mjf_efc_margin
    ?mjf_efc_frictionloss
    ?mjf_efc_diagApprox
    ?mjf_efc_KBIP
    ?mjf_efc_D
    ?mjf_efc_R
    ?mjf_efc_AR_rownnz
    ?mjf_efc_AR_rowadr
    ?mjf_efc_AR_colind
    ?mjf_efc_AR
    ?mjf_ten_velocity
    ?mjf_actuator_velocity
    ?mjf_cvel
    ?mjf_cdof_dot
    ?mjf_qfrc_bias
    ?mjf_qfrc_passive
    ?mjf_efc_vel
    ?mjf_efc_aref
    ?mjf_subtree_linvel
    ?mjf_subtree_angmom
    ?mjf_actuator_force
    ?mjf_qfrc_actuator
    ?mjf_qfrc_unc
    ?mjf_qacc_unc
    ?mjf_efc_b
    ?mjf_efc_force
    ?mjf_efc_state
    ?mjf_qfrc_constraint
    ?mjf_qfrc_inverse
    ?mjf_cacc
    ?mjf_cfrc_int
    ?mjf_cfrc_ext
    ()
  =
  let x = mjData_allocate () in
  Option.iter (mjData_set_nstack x) mjf_nstack;
  Option.iter (mjData_set_nbuffer x) mjf_nbuffer;
  Option.iter (mjData_set_pstack x) mjf_pstack;
  Option.iter (mjData_set_maxuse_stack x) mjf_maxuse_stack;
  Option.iter (mjData_set_maxuse_con x) mjf_maxuse_con;
  Option.iter (mjData_set_maxuse_efc x) mjf_maxuse_efc;
  Option.iter (mjData_set_warning x) mjf_warning;
  Option.iter (mjData_set_timer x) mjf_timer;
  Option.iter (mjData_set_solver x) mjf_solver;
  Option.iter (mjData_set_solver_iter x) mjf_solver_iter;
  Option.iter (mjData_set_solver_nnz x) mjf_solver_nnz;
  Option.iter (mjData_set_solver_fwdinv x) mjf_solver_fwdinv;
  Option.iter (mjData_set_ne x) mjf_ne;
  Option.iter (mjData_set_nf x) mjf_nf;
  Option.iter (mjData_set_nefc x) mjf_nefc;
  Option.iter (mjData_set_ncon x) mjf_ncon;
  Option.iter (mjData_set_time x) mjf_time;
  Option.iter (mjData_set_energy x) mjf_energy;
  Option.iter (mjData_set_buffer x) mjf_buffer;
  Option.iter (mjData_set_stack x) mjf_stack;
  Option.iter (mjData_set_qpos x) mjf_qpos;
  Option.iter (mjData_set_qvel x) mjf_qvel;
  Option.iter (mjData_set_act x) mjf_act;
  Option.iter (mjData_set_qacc_warmstart x) mjf_qacc_warmstart;
  Option.iter (mjData_set_ctrl x) mjf_ctrl;
  Option.iter (mjData_set_qfrc_applied x) mjf_qfrc_applied;
  Option.iter (mjData_set_xfrc_applied x) mjf_xfrc_applied;
  Option.iter (mjData_set_qacc x) mjf_qacc;
  Option.iter (mjData_set_act_dot x) mjf_act_dot;
  Option.iter (mjData_set_mocap_pos x) mjf_mocap_pos;
  Option.iter (mjData_set_mocap_quat x) mjf_mocap_quat;
  Option.iter (mjData_set_userdata x) mjf_userdata;
  Option.iter (mjData_set_sensordata x) mjf_sensordata;
  Option.iter (mjData_set_xpos x) mjf_xpos;
  Option.iter (mjData_set_xquat x) mjf_xquat;
  Option.iter (mjData_set_xmat x) mjf_xmat;
  Option.iter (mjData_set_xipos x) mjf_xipos;
  Option.iter (mjData_set_ximat x) mjf_ximat;
  Option.iter (mjData_set_xanchor x) mjf_xanchor;
  Option.iter (mjData_set_xaxis x) mjf_xaxis;
  Option.iter (mjData_set_geom_xpos x) mjf_geom_xpos;
  Option.iter (mjData_set_geom_xmat x) mjf_geom_xmat;
  Option.iter (mjData_set_site_xpos x) mjf_site_xpos;
  Option.iter (mjData_set_site_xmat x) mjf_site_xmat;
  Option.iter (mjData_set_cam_xpos x) mjf_cam_xpos;
  Option.iter (mjData_set_cam_xmat x) mjf_cam_xmat;
  Option.iter (mjData_set_light_xpos x) mjf_light_xpos;
  Option.iter (mjData_set_light_xdir x) mjf_light_xdir;
  Option.iter (mjData_set_subtree_com x) mjf_subtree_com;
  Option.iter (mjData_set_cdof x) mjf_cdof;
  Option.iter (mjData_set_cinert x) mjf_cinert;
  Option.iter (mjData_set_ten_wrapadr x) mjf_ten_wrapadr;
  Option.iter (mjData_set_ten_wrapnum x) mjf_ten_wrapnum;
  Option.iter (mjData_set_ten_J_rownnz x) mjf_ten_J_rownnz;
  Option.iter (mjData_set_ten_J_rowadr x) mjf_ten_J_rowadr;
  Option.iter (mjData_set_ten_J_colind x) mjf_ten_J_colind;
  Option.iter (mjData_set_ten_length x) mjf_ten_length;
  Option.iter (mjData_set_ten_J x) mjf_ten_J;
  Option.iter (mjData_set_wrap_obj x) mjf_wrap_obj;
  Option.iter (mjData_set_wrap_xpos x) mjf_wrap_xpos;
  Option.iter (mjData_set_actuator_length x) mjf_actuator_length;
  Option.iter (mjData_set_actuator_moment x) mjf_actuator_moment;
  Option.iter (mjData_set_crb x) mjf_crb;
  Option.iter (mjData_set_qM x) mjf_qM;
  Option.iter (mjData_set_qLD x) mjf_qLD;
  Option.iter (mjData_set_qLDiagInv x) mjf_qLDiagInv;
  Option.iter (mjData_set_qLDiagSqrtInv x) mjf_qLDiagSqrtInv;
  Option.iter (mjData_set_contact x) mjf_contact;
  Option.iter (mjData_set_efc_type x) mjf_efc_type;
  Option.iter (mjData_set_efc_id x) mjf_efc_id;
  Option.iter (mjData_set_efc_J_rownnz x) mjf_efc_J_rownnz;
  Option.iter (mjData_set_efc_J_rowadr x) mjf_efc_J_rowadr;
  Option.iter (mjData_set_efc_J_rowsuper x) mjf_efc_J_rowsuper;
  Option.iter (mjData_set_efc_J_colind x) mjf_efc_J_colind;
  Option.iter (mjData_set_efc_JT_rownnz x) mjf_efc_JT_rownnz;
  Option.iter (mjData_set_efc_JT_rowadr x) mjf_efc_JT_rowadr;
  Option.iter (mjData_set_efc_JT_rowsuper x) mjf_efc_JT_rowsuper;
  Option.iter (mjData_set_efc_JT_colind x) mjf_efc_JT_colind;
  Option.iter (mjData_set_efc_J x) mjf_efc_J;
  Option.iter (mjData_set_efc_JT x) mjf_efc_JT;
  Option.iter (mjData_set_efc_pos x) mjf_efc_pos;
  Option.iter (mjData_set_efc_margin x) mjf_efc_margin;
  Option.iter (mjData_set_efc_frictionloss x) mjf_efc_frictionloss;
  Option.iter (mjData_set_efc_diagApprox x) mjf_efc_diagApprox;
  Option.iter (mjData_set_efc_KBIP x) mjf_efc_KBIP;
  Option.iter (mjData_set_efc_D x) mjf_efc_D;
  Option.iter (mjData_set_efc_R x) mjf_efc_R;
  Option.iter (mjData_set_efc_AR_rownnz x) mjf_efc_AR_rownnz;
  Option.iter (mjData_set_efc_AR_rowadr x) mjf_efc_AR_rowadr;
  Option.iter (mjData_set_efc_AR_colind x) mjf_efc_AR_colind;
  Option.iter (mjData_set_efc_AR x) mjf_efc_AR;
  Option.iter (mjData_set_ten_velocity x) mjf_ten_velocity;
  Option.iter (mjData_set_actuator_velocity x) mjf_actuator_velocity;
  Option.iter (mjData_set_cvel x) mjf_cvel;
  Option.iter (mjData_set_cdof_dot x) mjf_cdof_dot;
  Option.iter (mjData_set_qfrc_bias x) mjf_qfrc_bias;
  Option.iter (mjData_set_qfrc_passive x) mjf_qfrc_passive;
  Option.iter (mjData_set_efc_vel x) mjf_efc_vel;
  Option.iter (mjData_set_efc_aref x) mjf_efc_aref;
  Option.iter (mjData_set_subtree_linvel x) mjf_subtree_linvel;
  Option.iter (mjData_set_subtree_angmom x) mjf_subtree_angmom;
  Option.iter (mjData_set_actuator_force x) mjf_actuator_force;
  Option.iter (mjData_set_qfrc_actuator x) mjf_qfrc_actuator;
  Option.iter (mjData_set_qfrc_unc x) mjf_qfrc_unc;
  Option.iter (mjData_set_qacc_unc x) mjf_qacc_unc;
  Option.iter (mjData_set_efc_b x) mjf_efc_b;
  Option.iter (mjData_set_efc_force x) mjf_efc_force;
  Option.iter (mjData_set_efc_state x) mjf_efc_state;
  Option.iter (mjData_set_qfrc_constraint x) mjf_qfrc_constraint;
  Option.iter (mjData_set_qfrc_inverse x) mjf_qfrc_inverse;
  Option.iter (mjData_set_cacc x) mjf_cacc;
  Option.iter (mjData_set_cfrc_int x) mjf_cfrc_int;
  Option.iter (mjData_set_cfrc_ext x) mjf_cfrc_ext;
  x


(** returns mjData carray from list of mjData *)
let mjData_to_carray = Ctypes.CArray.of_list Typs.mjData

(** convert mjtCatBit type to int *)
let mjtCatBit_to_int = function
  | MjCAT_STATIC  -> Typs.mjCAT_STATIC |> Int64.to_int
  | MjCAT_DYNAMIC -> Typs.mjCAT_DYNAMIC |> Int64.to_int
  | MjCAT_DECOR   -> Typs.mjCAT_DECOR |> Int64.to_int
  | MjCAT_ALL     -> Typs.mjCAT_ALL |> Int64.to_int


(** convert mjtMouse type to int *)
let mjtMouse_to_int = function
  | MjMOUSE_NONE     -> Typs.mjMOUSE_NONE |> Int64.to_int
  | MjMOUSE_ROTATE_V -> Typs.mjMOUSE_ROTATE_V |> Int64.to_int
  | MjMOUSE_ROTATE_H -> Typs.mjMOUSE_ROTATE_H |> Int64.to_int
  | MjMOUSE_MOVE_V   -> Typs.mjMOUSE_MOVE_V |> Int64.to_int
  | MjMOUSE_MOVE_H   -> Typs.mjMOUSE_MOVE_H |> Int64.to_int
  | MjMOUSE_ZOOM     -> Typs.mjMOUSE_ZOOM |> Int64.to_int
  | MjMOUSE_SELECT   -> Typs.mjMOUSE_SELECT |> Int64.to_int


(** convert mjtPertBit type to int *)
let mjtPertBit_to_int = function
  | MjPERT_TRANSLATE -> Typs.mjPERT_TRANSLATE |> Int64.to_int
  | MjPERT_ROTATE    -> Typs.mjPERT_ROTATE |> Int64.to_int


(** convert mjtCamera type to int *)
let mjtCamera_to_int = function
  | MjCAMERA_FREE     -> Typs.mjCAMERA_FREE |> Int64.to_int
  | MjCAMERA_TRACKING -> Typs.mjCAMERA_TRACKING |> Int64.to_int
  | MjCAMERA_FIXED    -> Typs.mjCAMERA_FIXED |> Int64.to_int
  | MjCAMERA_USER     -> Typs.mjCAMERA_USER |> Int64.to_int


(** convert mjtLabel type to int *)
let mjtLabel_to_int = function
  | MjLABEL_NONE         -> Typs.mjLABEL_NONE |> Int64.to_int
  | MjLABEL_BODY         -> Typs.mjLABEL_BODY |> Int64.to_int
  | MjLABEL_JOINT        -> Typs.mjLABEL_JOINT |> Int64.to_int
  | MjLABEL_GEOM         -> Typs.mjLABEL_GEOM |> Int64.to_int
  | MjLABEL_SITE         -> Typs.mjLABEL_SITE |> Int64.to_int
  | MjLABEL_CAMERA       -> Typs.mjLABEL_CAMERA |> Int64.to_int
  | MjLABEL_LIGHT        -> Typs.mjLABEL_LIGHT |> Int64.to_int
  | MjLABEL_TENDON       -> Typs.mjLABEL_TENDON |> Int64.to_int
  | MjLABEL_ACTUATOR     -> Typs.mjLABEL_ACTUATOR |> Int64.to_int
  | MjLABEL_CONSTRAINT   -> Typs.mjLABEL_CONSTRAINT |> Int64.to_int
  | MjLABEL_SKIN         -> Typs.mjLABEL_SKIN |> Int64.to_int
  | MjLABEL_SELECTION    -> Typs.mjLABEL_SELECTION |> Int64.to_int
  | MjLABEL_SELPNT       -> Typs.mjLABEL_SELPNT |> Int64.to_int
  | MjLABEL_CONTACTFORCE -> Typs.mjLABEL_CONTACTFORCE |> Int64.to_int
  | MjNLABEL             -> Typs.mjNLABEL |> Int64.to_int


(** convert mjtFrame type to int *)
let mjtFrame_to_int = function
  | MjFRAME_NONE   -> Typs.mjFRAME_NONE |> Int64.to_int
  | MjFRAME_BODY   -> Typs.mjFRAME_BODY |> Int64.to_int
  | MjFRAME_GEOM   -> Typs.mjFRAME_GEOM |> Int64.to_int
  | MjFRAME_SITE   -> Typs.mjFRAME_SITE |> Int64.to_int
  | MjFRAME_CAMERA -> Typs.mjFRAME_CAMERA |> Int64.to_int
  | MjFRAME_LIGHT  -> Typs.mjFRAME_LIGHT |> Int64.to_int
  | MjFRAME_WORLD  -> Typs.mjFRAME_WORLD |> Int64.to_int
  | MjNFRAME       -> Typs.mjNFRAME |> Int64.to_int


(** convert mjtVisFlag type to int *)
let mjtVisFlag_to_int = function
  | MjVIS_CONVEXHULL   -> Typs.mjVIS_CONVEXHULL |> Int64.to_int
  | MjVIS_TEXTURE      -> Typs.mjVIS_TEXTURE |> Int64.to_int
  | MjVIS_JOINT        -> Typs.mjVIS_JOINT |> Int64.to_int
  | MjVIS_ACTUATOR     -> Typs.mjVIS_ACTUATOR |> Int64.to_int
  | MjVIS_CAMERA       -> Typs.mjVIS_CAMERA |> Int64.to_int
  | MjVIS_LIGHT        -> Typs.mjVIS_LIGHT |> Int64.to_int
  | MjVIS_TENDON       -> Typs.mjVIS_TENDON |> Int64.to_int
  | MjVIS_RANGEFINDER  -> Typs.mjVIS_RANGEFINDER |> Int64.to_int
  | MjVIS_CONSTRAINT   -> Typs.mjVIS_CONSTRAINT |> Int64.to_int
  | MjVIS_INERTIA      -> Typs.mjVIS_INERTIA |> Int64.to_int
  | MjVIS_SCLINERTIA   -> Typs.mjVIS_SCLINERTIA |> Int64.to_int
  | MjVIS_PERTFORCE    -> Typs.mjVIS_PERTFORCE |> Int64.to_int
  | MjVIS_PERTOBJ      -> Typs.mjVIS_PERTOBJ |> Int64.to_int
  | MjVIS_CONTACTPOINT -> Typs.mjVIS_CONTACTPOINT |> Int64.to_int
  | MjVIS_CONTACTFORCE -> Typs.mjVIS_CONTACTFORCE |> Int64.to_int
  | MjVIS_CONTACTSPLIT -> Typs.mjVIS_CONTACTSPLIT |> Int64.to_int
  | MjVIS_TRANSPARENT  -> Typs.mjVIS_TRANSPARENT |> Int64.to_int
  | MjVIS_AUTOCONNECT  -> Typs.mjVIS_AUTOCONNECT |> Int64.to_int
  | MjVIS_COM          -> Typs.mjVIS_COM |> Int64.to_int
  | MjVIS_SELECT       -> Typs.mjVIS_SELECT |> Int64.to_int
  | MjVIS_STATIC       -> Typs.mjVIS_STATIC |> Int64.to_int
  | MjVIS_SKIN         -> Typs.mjVIS_SKIN |> Int64.to_int
  | MjNVISFLAG         -> Typs.mjNVISFLAG |> Int64.to_int


(** convert mjtRndFlag type to int *)
let mjtRndFlag_to_int = function
  | MjRND_SHADOW     -> Typs.mjRND_SHADOW |> Int64.to_int
  | MjRND_WIREFRAME  -> Typs.mjRND_WIREFRAME |> Int64.to_int
  | MjRND_REFLECTION -> Typs.mjRND_REFLECTION |> Int64.to_int
  | MjRND_ADDITIVE   -> Typs.mjRND_ADDITIVE |> Int64.to_int
  | MjRND_SKYBOX     -> Typs.mjRND_SKYBOX |> Int64.to_int
  | MjRND_FOG        -> Typs.mjRND_FOG |> Int64.to_int
  | MjRND_HAZE       -> Typs.mjRND_HAZE |> Int64.to_int
  | MjRND_SEGMENT    -> Typs.mjRND_SEGMENT |> Int64.to_int
  | MjRND_IDCOLOR    -> Typs.mjRND_IDCOLOR |> Int64.to_int
  | MjNRNDFLAG       -> Typs.mjNRNDFLAG |> Int64.to_int


(** convert mjtStereo type to int *)
let mjtStereo_to_int = function
  | MjSTEREO_NONE         -> Typs.mjSTEREO_NONE |> Int64.to_int
  | MjSTEREO_QUADBUFFERED -> Typs.mjSTEREO_QUADBUFFERED |> Int64.to_int
  | MjSTEREO_SIDEBYSIDE   -> Typs.mjSTEREO_SIDEBYSIDE |> Int64.to_int


(** allocate fresh mjvPerturb struct *)
let mjvPerturb_allocate () = Ctypes.(make Typs.mjvPerturb)

(** make null mjvPerturb struct ptr *)
let mjvPerturb_null () = Ctypes.(from_voidp Typs.mjvPerturb null)

(** get select from mjvPerturb *)
let mjvPerturb_get_select x = Ctypes.(getf x Typs.mjvPerturb_select)

(** set select for mjvPerturb *)
let mjvPerturb_set_select x y = Ctypes.(setf x Typs.mjvPerturb_select y)

(** get skinselect from mjvPerturb *)
let mjvPerturb_get_skinselect x = Ctypes.(getf x Typs.mjvPerturb_skinselect)

(** set skinselect for mjvPerturb *)
let mjvPerturb_set_skinselect x y = Ctypes.(setf x Typs.mjvPerturb_skinselect y)

(** get active from mjvPerturb *)
let mjvPerturb_get_active x = Ctypes.(getf x Typs.mjvPerturb_active)

(** set active for mjvPerturb *)
let mjvPerturb_set_active x y = Ctypes.(setf x Typs.mjvPerturb_active y)

(** get active2 from mjvPerturb *)
let mjvPerturb_get_active2 x = Ctypes.(getf x Typs.mjvPerturb_active2)

(** set active2 for mjvPerturb *)
let mjvPerturb_set_active2 x y = Ctypes.(setf x Typs.mjvPerturb_active2 y)

(** get refpos from mjvPerturb *)
let mjvPerturb_get_refpos x = Ctypes.(getf x Typs.mjvPerturb_refpos)

(** set refpos for mjvPerturb *)
let mjvPerturb_set_refpos x y = Ctypes.(setf x Typs.mjvPerturb_refpos y)

(** get refquat from mjvPerturb *)
let mjvPerturb_get_refquat x = Ctypes.(getf x Typs.mjvPerturb_refquat)

(** set refquat for mjvPerturb *)
let mjvPerturb_set_refquat x y = Ctypes.(setf x Typs.mjvPerturb_refquat y)

(** get localpos from mjvPerturb *)
let mjvPerturb_get_localpos x = Ctypes.(getf x Typs.mjvPerturb_localpos)

(** set localpos for mjvPerturb *)
let mjvPerturb_set_localpos x y = Ctypes.(setf x Typs.mjvPerturb_localpos y)

(** get scale from mjvPerturb *)
let mjvPerturb_get_scale x = Ctypes.(getf x Typs.mjvPerturb_scale)

(** set scale for mjvPerturb *)
let mjvPerturb_set_scale x y = Ctypes.(setf x Typs.mjvPerturb_scale y)

(** make mjvPerturb struct *)
let mjvPerturb_make
    ?mjf_select
    ?mjf_skinselect
    ?mjf_active
    ?mjf_active2
    ?mjf_refpos
    ?mjf_refquat
    ?mjf_localpos
    ?mjf_scale
    ()
  =
  let x = mjvPerturb_allocate () in
  Option.iter (mjvPerturb_set_select x) mjf_select;
  Option.iter (mjvPerturb_set_skinselect x) mjf_skinselect;
  Option.iter (mjvPerturb_set_active x) mjf_active;
  Option.iter (mjvPerturb_set_active2 x) mjf_active2;
  Option.iter (mjvPerturb_set_refpos x) mjf_refpos;
  Option.iter (mjvPerturb_set_refquat x) mjf_refquat;
  Option.iter (mjvPerturb_set_localpos x) mjf_localpos;
  Option.iter (mjvPerturb_set_scale x) mjf_scale;
  x


(** returns mjvPerturb carray from list of mjvPerturb *)
let mjvPerturb_to_carray = Ctypes.CArray.of_list Typs.mjvPerturb

(** allocate fresh mjvCamera struct *)
let mjvCamera_allocate () = Ctypes.(make Typs.mjvCamera)

(** make null mjvCamera struct ptr *)
let mjvCamera_null () = Ctypes.(from_voidp Typs.mjvCamera null)

(** get type from mjvCamera *)
let mjvCamera_get_type x = Ctypes.(getf x Typs.mjvCamera_type)

(** set type for mjvCamera *)
let mjvCamera_set_type x y = Ctypes.(setf x Typs.mjvCamera_type y)

(** get fixedcamid from mjvCamera *)
let mjvCamera_get_fixedcamid x = Ctypes.(getf x Typs.mjvCamera_fixedcamid)

(** set fixedcamid for mjvCamera *)
let mjvCamera_set_fixedcamid x y = Ctypes.(setf x Typs.mjvCamera_fixedcamid y)

(** get trackbodyid from mjvCamera *)
let mjvCamera_get_trackbodyid x = Ctypes.(getf x Typs.mjvCamera_trackbodyid)

(** set trackbodyid for mjvCamera *)
let mjvCamera_set_trackbodyid x y = Ctypes.(setf x Typs.mjvCamera_trackbodyid y)

(** get lookat from mjvCamera *)
let mjvCamera_get_lookat x = Ctypes.(getf x Typs.mjvCamera_lookat)

(** set lookat for mjvCamera *)
let mjvCamera_set_lookat x y = Ctypes.(setf x Typs.mjvCamera_lookat y)

(** get distance from mjvCamera *)
let mjvCamera_get_distance x = Ctypes.(getf x Typs.mjvCamera_distance)

(** set distance for mjvCamera *)
let mjvCamera_set_distance x y = Ctypes.(setf x Typs.mjvCamera_distance y)

(** get azimuth from mjvCamera *)
let mjvCamera_get_azimuth x = Ctypes.(getf x Typs.mjvCamera_azimuth)

(** set azimuth for mjvCamera *)
let mjvCamera_set_azimuth x y = Ctypes.(setf x Typs.mjvCamera_azimuth y)

(** get elevation from mjvCamera *)
let mjvCamera_get_elevation x = Ctypes.(getf x Typs.mjvCamera_elevation)

(** set elevation for mjvCamera *)
let mjvCamera_set_elevation x y = Ctypes.(setf x Typs.mjvCamera_elevation y)

(** make mjvCamera struct *)
let mjvCamera_make
    ?mjf_type
    ?mjf_fixedcamid
    ?mjf_trackbodyid
    ?mjf_lookat
    ?mjf_distance
    ?mjf_azimuth
    ?mjf_elevation
    ()
  =
  let x = mjvCamera_allocate () in
  Option.iter (mjvCamera_set_type x) mjf_type;
  Option.iter (mjvCamera_set_fixedcamid x) mjf_fixedcamid;
  Option.iter (mjvCamera_set_trackbodyid x) mjf_trackbodyid;
  Option.iter (mjvCamera_set_lookat x) mjf_lookat;
  Option.iter (mjvCamera_set_distance x) mjf_distance;
  Option.iter (mjvCamera_set_azimuth x) mjf_azimuth;
  Option.iter (mjvCamera_set_elevation x) mjf_elevation;
  x


(** returns mjvCamera carray from list of mjvCamera *)
let mjvCamera_to_carray = Ctypes.CArray.of_list Typs.mjvCamera

(** allocate fresh mjvGLCamera struct *)
let mjvGLCamera_allocate () = Ctypes.(make Typs.mjvGLCamera)

(** make null mjvGLCamera struct ptr *)
let mjvGLCamera_null () = Ctypes.(from_voidp Typs.mjvGLCamera null)

(** get pos from mjvGLCamera *)
let mjvGLCamera_get_pos x = Ctypes.(getf x Typs.mjvGLCamera_pos)

(** set pos for mjvGLCamera *)
let mjvGLCamera_set_pos x y = Ctypes.(setf x Typs.mjvGLCamera_pos y)

(** get forward from mjvGLCamera *)
let mjvGLCamera_get_forward x = Ctypes.(getf x Typs.mjvGLCamera_forward)

(** set forward for mjvGLCamera *)
let mjvGLCamera_set_forward x y = Ctypes.(setf x Typs.mjvGLCamera_forward y)

(** get up from mjvGLCamera *)
let mjvGLCamera_get_up x = Ctypes.(getf x Typs.mjvGLCamera_up)

(** set up for mjvGLCamera *)
let mjvGLCamera_set_up x y = Ctypes.(setf x Typs.mjvGLCamera_up y)

(** get frustum_center from mjvGLCamera *)
let mjvGLCamera_get_frustum_center x = Ctypes.(getf x Typs.mjvGLCamera_frustum_center)

(** set frustum_center for mjvGLCamera *)
let mjvGLCamera_set_frustum_center x y = Ctypes.(setf x Typs.mjvGLCamera_frustum_center y)

(** get frustum_bottom from mjvGLCamera *)
let mjvGLCamera_get_frustum_bottom x = Ctypes.(getf x Typs.mjvGLCamera_frustum_bottom)

(** set frustum_bottom for mjvGLCamera *)
let mjvGLCamera_set_frustum_bottom x y = Ctypes.(setf x Typs.mjvGLCamera_frustum_bottom y)

(** get frustum_top from mjvGLCamera *)
let mjvGLCamera_get_frustum_top x = Ctypes.(getf x Typs.mjvGLCamera_frustum_top)

(** set frustum_top for mjvGLCamera *)
let mjvGLCamera_set_frustum_top x y = Ctypes.(setf x Typs.mjvGLCamera_frustum_top y)

(** get frustum_near from mjvGLCamera *)
let mjvGLCamera_get_frustum_near x = Ctypes.(getf x Typs.mjvGLCamera_frustum_near)

(** set frustum_near for mjvGLCamera *)
let mjvGLCamera_set_frustum_near x y = Ctypes.(setf x Typs.mjvGLCamera_frustum_near y)

(** get frustum_far from mjvGLCamera *)
let mjvGLCamera_get_frustum_far x = Ctypes.(getf x Typs.mjvGLCamera_frustum_far)

(** set frustum_far for mjvGLCamera *)
let mjvGLCamera_set_frustum_far x y = Ctypes.(setf x Typs.mjvGLCamera_frustum_far y)

(** make mjvGLCamera struct *)
let mjvGLCamera_make
    ?mjf_pos
    ?mjf_forward
    ?mjf_up
    ?mjf_frustum_center
    ?mjf_frustum_bottom
    ?mjf_frustum_top
    ?mjf_frustum_near
    ?mjf_frustum_far
    ()
  =
  let x = mjvGLCamera_allocate () in
  Option.iter (mjvGLCamera_set_pos x) mjf_pos;
  Option.iter (mjvGLCamera_set_forward x) mjf_forward;
  Option.iter (mjvGLCamera_set_up x) mjf_up;
  Option.iter (mjvGLCamera_set_frustum_center x) mjf_frustum_center;
  Option.iter (mjvGLCamera_set_frustum_bottom x) mjf_frustum_bottom;
  Option.iter (mjvGLCamera_set_frustum_top x) mjf_frustum_top;
  Option.iter (mjvGLCamera_set_frustum_near x) mjf_frustum_near;
  Option.iter (mjvGLCamera_set_frustum_far x) mjf_frustum_far;
  x


(** returns mjvGLCamera carray from list of mjvGLCamera *)
let mjvGLCamera_to_carray = Ctypes.CArray.of_list Typs.mjvGLCamera

(** allocate fresh mjvGeom struct *)
let mjvGeom_allocate () = Ctypes.(make Typs.mjvGeom)

(** make null mjvGeom struct ptr *)
let mjvGeom_null () = Ctypes.(from_voidp Typs.mjvGeom null)

(** get type from mjvGeom *)
let mjvGeom_get_type x = Ctypes.(getf x Typs.mjvGeom_type)

(** set type for mjvGeom *)
let mjvGeom_set_type x y = Ctypes.(setf x Typs.mjvGeom_type y)

(** get dataid from mjvGeom *)
let mjvGeom_get_dataid x = Ctypes.(getf x Typs.mjvGeom_dataid)

(** set dataid for mjvGeom *)
let mjvGeom_set_dataid x y = Ctypes.(setf x Typs.mjvGeom_dataid y)

(** get objtype from mjvGeom *)
let mjvGeom_get_objtype x = Ctypes.(getf x Typs.mjvGeom_objtype)

(** set objtype for mjvGeom *)
let mjvGeom_set_objtype x y = Ctypes.(setf x Typs.mjvGeom_objtype y)

(** get objid from mjvGeom *)
let mjvGeom_get_objid x = Ctypes.(getf x Typs.mjvGeom_objid)

(** set objid for mjvGeom *)
let mjvGeom_set_objid x y = Ctypes.(setf x Typs.mjvGeom_objid y)

(** get category from mjvGeom *)
let mjvGeom_get_category x = Ctypes.(getf x Typs.mjvGeom_category)

(** set category for mjvGeom *)
let mjvGeom_set_category x y = Ctypes.(setf x Typs.mjvGeom_category y)

(** get texid from mjvGeom *)
let mjvGeom_get_texid x = Ctypes.(getf x Typs.mjvGeom_texid)

(** set texid for mjvGeom *)
let mjvGeom_set_texid x y = Ctypes.(setf x Typs.mjvGeom_texid y)

(** get texuniform from mjvGeom *)
let mjvGeom_get_texuniform x = Ctypes.(getf x Typs.mjvGeom_texuniform)

(** set texuniform for mjvGeom *)
let mjvGeom_set_texuniform x y = Ctypes.(setf x Typs.mjvGeom_texuniform y)

(** get texcoord from mjvGeom *)
let mjvGeom_get_texcoord x = Ctypes.(getf x Typs.mjvGeom_texcoord)

(** set texcoord for mjvGeom *)
let mjvGeom_set_texcoord x y = Ctypes.(setf x Typs.mjvGeom_texcoord y)

(** get segid from mjvGeom *)
let mjvGeom_get_segid x = Ctypes.(getf x Typs.mjvGeom_segid)

(** set segid for mjvGeom *)
let mjvGeom_set_segid x y = Ctypes.(setf x Typs.mjvGeom_segid y)

(** get texrepeat from mjvGeom *)
let mjvGeom_get_texrepeat x = Ctypes.(getf x Typs.mjvGeom_texrepeat)

(** set texrepeat for mjvGeom *)
let mjvGeom_set_texrepeat x y = Ctypes.(setf x Typs.mjvGeom_texrepeat y)

(** get size from mjvGeom *)
let mjvGeom_get_size x = Ctypes.(getf x Typs.mjvGeom_size)

(** set size for mjvGeom *)
let mjvGeom_set_size x y = Ctypes.(setf x Typs.mjvGeom_size y)

(** get pos from mjvGeom *)
let mjvGeom_get_pos x = Ctypes.(getf x Typs.mjvGeom_pos)

(** set pos for mjvGeom *)
let mjvGeom_set_pos x y = Ctypes.(setf x Typs.mjvGeom_pos y)

(** get mat from mjvGeom *)
let mjvGeom_get_mat x = Ctypes.(getf x Typs.mjvGeom_mat)

(** set mat for mjvGeom *)
let mjvGeom_set_mat x y = Ctypes.(setf x Typs.mjvGeom_mat y)

(** get rgba from mjvGeom *)
let mjvGeom_get_rgba x = Ctypes.(getf x Typs.mjvGeom_rgba)

(** set rgba for mjvGeom *)
let mjvGeom_set_rgba x y = Ctypes.(setf x Typs.mjvGeom_rgba y)

(** get emission from mjvGeom *)
let mjvGeom_get_emission x = Ctypes.(getf x Typs.mjvGeom_emission)

(** set emission for mjvGeom *)
let mjvGeom_set_emission x y = Ctypes.(setf x Typs.mjvGeom_emission y)

(** get specular from mjvGeom *)
let mjvGeom_get_specular x = Ctypes.(getf x Typs.mjvGeom_specular)

(** set specular for mjvGeom *)
let mjvGeom_set_specular x y = Ctypes.(setf x Typs.mjvGeom_specular y)

(** get shininess from mjvGeom *)
let mjvGeom_get_shininess x = Ctypes.(getf x Typs.mjvGeom_shininess)

(** set shininess for mjvGeom *)
let mjvGeom_set_shininess x y = Ctypes.(setf x Typs.mjvGeom_shininess y)

(** get reflectance from mjvGeom *)
let mjvGeom_get_reflectance x = Ctypes.(getf x Typs.mjvGeom_reflectance)

(** set reflectance for mjvGeom *)
let mjvGeom_set_reflectance x y = Ctypes.(setf x Typs.mjvGeom_reflectance y)

(** get label from mjvGeom *)
let mjvGeom_get_label x = Ctypes.(getf x Typs.mjvGeom_label)

(** set label for mjvGeom *)
let mjvGeom_set_label x y = Ctypes.(setf x Typs.mjvGeom_label y)

(** get camdist from mjvGeom *)
let mjvGeom_get_camdist x = Ctypes.(getf x Typs.mjvGeom_camdist)

(** set camdist for mjvGeom *)
let mjvGeom_set_camdist x y = Ctypes.(setf x Typs.mjvGeom_camdist y)

(** get modelrbound from mjvGeom *)
let mjvGeom_get_modelrbound x = Ctypes.(getf x Typs.mjvGeom_modelrbound)

(** set modelrbound for mjvGeom *)
let mjvGeom_set_modelrbound x y = Ctypes.(setf x Typs.mjvGeom_modelrbound y)

(** get transparent from mjvGeom *)
let mjvGeom_get_transparent x = Ctypes.(getf x Typs.mjvGeom_transparent)

(** set transparent for mjvGeom *)
let mjvGeom_set_transparent x y = Ctypes.(setf x Typs.mjvGeom_transparent y)

(** make mjvGeom struct *)
let mjvGeom_make
    ?mjf_type
    ?mjf_dataid
    ?mjf_objtype
    ?mjf_objid
    ?mjf_category
    ?mjf_texid
    ?mjf_texuniform
    ?mjf_texcoord
    ?mjf_segid
    ?mjf_texrepeat
    ?mjf_size
    ?mjf_pos
    ?mjf_mat
    ?mjf_rgba
    ?mjf_emission
    ?mjf_specular
    ?mjf_shininess
    ?mjf_reflectance
    ?mjf_label
    ?mjf_camdist
    ?mjf_modelrbound
    ?mjf_transparent
    ()
  =
  let x = mjvGeom_allocate () in
  Option.iter (mjvGeom_set_type x) mjf_type;
  Option.iter (mjvGeom_set_dataid x) mjf_dataid;
  Option.iter (mjvGeom_set_objtype x) mjf_objtype;
  Option.iter (mjvGeom_set_objid x) mjf_objid;
  Option.iter (mjvGeom_set_category x) mjf_category;
  Option.iter (mjvGeom_set_texid x) mjf_texid;
  Option.iter (mjvGeom_set_texuniform x) mjf_texuniform;
  Option.iter (mjvGeom_set_texcoord x) mjf_texcoord;
  Option.iter (mjvGeom_set_segid x) mjf_segid;
  Option.iter (mjvGeom_set_texrepeat x) mjf_texrepeat;
  Option.iter (mjvGeom_set_size x) mjf_size;
  Option.iter (mjvGeom_set_pos x) mjf_pos;
  Option.iter (mjvGeom_set_mat x) mjf_mat;
  Option.iter (mjvGeom_set_rgba x) mjf_rgba;
  Option.iter (mjvGeom_set_emission x) mjf_emission;
  Option.iter (mjvGeom_set_specular x) mjf_specular;
  Option.iter (mjvGeom_set_shininess x) mjf_shininess;
  Option.iter (mjvGeom_set_reflectance x) mjf_reflectance;
  Option.iter (mjvGeom_set_label x) mjf_label;
  Option.iter (mjvGeom_set_camdist x) mjf_camdist;
  Option.iter (mjvGeom_set_modelrbound x) mjf_modelrbound;
  Option.iter (mjvGeom_set_transparent x) mjf_transparent;
  x


(** returns mjvGeom carray from list of mjvGeom *)
let mjvGeom_to_carray = Ctypes.CArray.of_list Typs.mjvGeom

(** allocate fresh mjvLight struct *)
let mjvLight_allocate () = Ctypes.(make Typs.mjvLight)

(** make null mjvLight struct ptr *)
let mjvLight_null () = Ctypes.(from_voidp Typs.mjvLight null)

(** get pos from mjvLight *)
let mjvLight_get_pos x = Ctypes.(getf x Typs.mjvLight_pos)

(** set pos for mjvLight *)
let mjvLight_set_pos x y = Ctypes.(setf x Typs.mjvLight_pos y)

(** get dir from mjvLight *)
let mjvLight_get_dir x = Ctypes.(getf x Typs.mjvLight_dir)

(** set dir for mjvLight *)
let mjvLight_set_dir x y = Ctypes.(setf x Typs.mjvLight_dir y)

(** get attenuation from mjvLight *)
let mjvLight_get_attenuation x = Ctypes.(getf x Typs.mjvLight_attenuation)

(** set attenuation for mjvLight *)
let mjvLight_set_attenuation x y = Ctypes.(setf x Typs.mjvLight_attenuation y)

(** get cutoff from mjvLight *)
let mjvLight_get_cutoff x = Ctypes.(getf x Typs.mjvLight_cutoff)

(** set cutoff for mjvLight *)
let mjvLight_set_cutoff x y = Ctypes.(setf x Typs.mjvLight_cutoff y)

(** get exponent from mjvLight *)
let mjvLight_get_exponent x = Ctypes.(getf x Typs.mjvLight_exponent)

(** set exponent for mjvLight *)
let mjvLight_set_exponent x y = Ctypes.(setf x Typs.mjvLight_exponent y)

(** get ambient from mjvLight *)
let mjvLight_get_ambient x = Ctypes.(getf x Typs.mjvLight_ambient)

(** set ambient for mjvLight *)
let mjvLight_set_ambient x y = Ctypes.(setf x Typs.mjvLight_ambient y)

(** get diffuse from mjvLight *)
let mjvLight_get_diffuse x = Ctypes.(getf x Typs.mjvLight_diffuse)

(** set diffuse for mjvLight *)
let mjvLight_set_diffuse x y = Ctypes.(setf x Typs.mjvLight_diffuse y)

(** get specular from mjvLight *)
let mjvLight_get_specular x = Ctypes.(getf x Typs.mjvLight_specular)

(** set specular for mjvLight *)
let mjvLight_set_specular x y = Ctypes.(setf x Typs.mjvLight_specular y)

(** get headlight from mjvLight *)
let mjvLight_get_headlight x = Ctypes.(getf x Typs.mjvLight_headlight)

(** set headlight for mjvLight *)
let mjvLight_set_headlight x y = Ctypes.(setf x Typs.mjvLight_headlight y)

(** get directional from mjvLight *)
let mjvLight_get_directional x = Ctypes.(getf x Typs.mjvLight_directional)

(** set directional for mjvLight *)
let mjvLight_set_directional x y = Ctypes.(setf x Typs.mjvLight_directional y)

(** get castshadow from mjvLight *)
let mjvLight_get_castshadow x = Ctypes.(getf x Typs.mjvLight_castshadow)

(** set castshadow for mjvLight *)
let mjvLight_set_castshadow x y = Ctypes.(setf x Typs.mjvLight_castshadow y)

(** make mjvLight struct *)
let mjvLight_make
    ?mjf_pos
    ?mjf_dir
    ?mjf_attenuation
    ?mjf_cutoff
    ?mjf_exponent
    ?mjf_ambient
    ?mjf_diffuse
    ?mjf_specular
    ?mjf_headlight
    ?mjf_directional
    ?mjf_castshadow
    ()
  =
  let x = mjvLight_allocate () in
  Option.iter (mjvLight_set_pos x) mjf_pos;
  Option.iter (mjvLight_set_dir x) mjf_dir;
  Option.iter (mjvLight_set_attenuation x) mjf_attenuation;
  Option.iter (mjvLight_set_cutoff x) mjf_cutoff;
  Option.iter (mjvLight_set_exponent x) mjf_exponent;
  Option.iter (mjvLight_set_ambient x) mjf_ambient;
  Option.iter (mjvLight_set_diffuse x) mjf_diffuse;
  Option.iter (mjvLight_set_specular x) mjf_specular;
  Option.iter (mjvLight_set_headlight x) mjf_headlight;
  Option.iter (mjvLight_set_directional x) mjf_directional;
  Option.iter (mjvLight_set_castshadow x) mjf_castshadow;
  x


(** returns mjvLight carray from list of mjvLight *)
let mjvLight_to_carray = Ctypes.CArray.of_list Typs.mjvLight

(** allocate fresh mjvOption struct *)
let mjvOption_allocate () = Ctypes.(make Typs.mjvOption)

(** make null mjvOption struct ptr *)
let mjvOption_null () = Ctypes.(from_voidp Typs.mjvOption null)

(** get label from mjvOption *)
let mjvOption_get_label x = Ctypes.(getf x Typs.mjvOption_label)

(** set label for mjvOption *)
let mjvOption_set_label x y = Ctypes.(setf x Typs.mjvOption_label y)

(** get frame from mjvOption *)
let mjvOption_get_frame x = Ctypes.(getf x Typs.mjvOption_frame)

(** set frame for mjvOption *)
let mjvOption_set_frame x y = Ctypes.(setf x Typs.mjvOption_frame y)

(** get geomgroup from mjvOption *)
let mjvOption_get_geomgroup x = Ctypes.(getf x Typs.mjvOption_geomgroup)

(** set geomgroup for mjvOption *)
let mjvOption_set_geomgroup x y = Ctypes.(setf x Typs.mjvOption_geomgroup y)

(** get sitegroup from mjvOption *)
let mjvOption_get_sitegroup x = Ctypes.(getf x Typs.mjvOption_sitegroup)

(** set sitegroup for mjvOption *)
let mjvOption_set_sitegroup x y = Ctypes.(setf x Typs.mjvOption_sitegroup y)

(** get jointgroup from mjvOption *)
let mjvOption_get_jointgroup x = Ctypes.(getf x Typs.mjvOption_jointgroup)

(** set jointgroup for mjvOption *)
let mjvOption_set_jointgroup x y = Ctypes.(setf x Typs.mjvOption_jointgroup y)

(** get tendongroup from mjvOption *)
let mjvOption_get_tendongroup x = Ctypes.(getf x Typs.mjvOption_tendongroup)

(** set tendongroup for mjvOption *)
let mjvOption_set_tendongroup x y = Ctypes.(setf x Typs.mjvOption_tendongroup y)

(** get actuatorgroup from mjvOption *)
let mjvOption_get_actuatorgroup x = Ctypes.(getf x Typs.mjvOption_actuatorgroup)

(** set actuatorgroup for mjvOption *)
let mjvOption_set_actuatorgroup x y = Ctypes.(setf x Typs.mjvOption_actuatorgroup y)

(** get flags from mjvOption *)
let mjvOption_get_flags x = Ctypes.(getf x Typs.mjvOption_flags)

(** set flags for mjvOption *)
let mjvOption_set_flags x y = Ctypes.(setf x Typs.mjvOption_flags y)

(** make mjvOption struct *)
let mjvOption_make
    ?mjf_label
    ?mjf_frame
    ?mjf_geomgroup
    ?mjf_sitegroup
    ?mjf_jointgroup
    ?mjf_tendongroup
    ?mjf_actuatorgroup
    ?mjf_flags
    ()
  =
  let x = mjvOption_allocate () in
  Option.iter (mjvOption_set_label x) mjf_label;
  Option.iter (mjvOption_set_frame x) mjf_frame;
  Option.iter (mjvOption_set_geomgroup x) mjf_geomgroup;
  Option.iter (mjvOption_set_sitegroup x) mjf_sitegroup;
  Option.iter (mjvOption_set_jointgroup x) mjf_jointgroup;
  Option.iter (mjvOption_set_tendongroup x) mjf_tendongroup;
  Option.iter (mjvOption_set_actuatorgroup x) mjf_actuatorgroup;
  Option.iter (mjvOption_set_flags x) mjf_flags;
  x


(** returns mjvOption carray from list of mjvOption *)
let mjvOption_to_carray = Ctypes.CArray.of_list Typs.mjvOption

(** allocate fresh mjvScene struct *)
let mjvScene_allocate () = Ctypes.(make Typs.mjvScene)

(** make null mjvScene struct ptr *)
let mjvScene_null () = Ctypes.(from_voidp Typs.mjvScene null)

(** get maxgeom from mjvScene *)
let mjvScene_get_maxgeom x = Ctypes.(getf x Typs.mjvScene_maxgeom)

(** set maxgeom for mjvScene *)
let mjvScene_set_maxgeom x y = Ctypes.(setf x Typs.mjvScene_maxgeom y)

(** get ngeom from mjvScene *)
let mjvScene_get_ngeom x = Ctypes.(getf x Typs.mjvScene_ngeom)

(** set ngeom for mjvScene *)
let mjvScene_set_ngeom x y = Ctypes.(setf x Typs.mjvScene_ngeom y)

(** get geoms from mjvScene *)
let mjvScene_get_geoms x = Ctypes.(getf x Typs.mjvScene_geoms)

(** set geoms for mjvScene *)
let mjvScene_set_geoms x y = Ctypes.(setf x Typs.mjvScene_geoms y)

(** get geomorder from mjvScene *)
let mjvScene_get_geomorder x = Ctypes.(getf x Typs.mjvScene_geomorder)

(** set geomorder for mjvScene *)
let mjvScene_set_geomorder x y = Ctypes.(setf x Typs.mjvScene_geomorder y)

(** get nskin from mjvScene *)
let mjvScene_get_nskin x = Ctypes.(getf x Typs.mjvScene_nskin)

(** set nskin for mjvScene *)
let mjvScene_set_nskin x y = Ctypes.(setf x Typs.mjvScene_nskin y)

(** get skinfacenum from mjvScene *)
let mjvScene_get_skinfacenum x = Ctypes.(getf x Typs.mjvScene_skinfacenum)

(** set skinfacenum for mjvScene *)
let mjvScene_set_skinfacenum x y = Ctypes.(setf x Typs.mjvScene_skinfacenum y)

(** get skinvertadr from mjvScene *)
let mjvScene_get_skinvertadr x = Ctypes.(getf x Typs.mjvScene_skinvertadr)

(** set skinvertadr for mjvScene *)
let mjvScene_set_skinvertadr x y = Ctypes.(setf x Typs.mjvScene_skinvertadr y)

(** get skinvertnum from mjvScene *)
let mjvScene_get_skinvertnum x = Ctypes.(getf x Typs.mjvScene_skinvertnum)

(** set skinvertnum for mjvScene *)
let mjvScene_set_skinvertnum x y = Ctypes.(setf x Typs.mjvScene_skinvertnum y)

(** get skinvert from mjvScene *)
let mjvScene_get_skinvert x = Ctypes.(getf x Typs.mjvScene_skinvert)

(** set skinvert for mjvScene *)
let mjvScene_set_skinvert x y = Ctypes.(setf x Typs.mjvScene_skinvert y)

(** get skinnormal from mjvScene *)
let mjvScene_get_skinnormal x = Ctypes.(getf x Typs.mjvScene_skinnormal)

(** set skinnormal for mjvScene *)
let mjvScene_set_skinnormal x y = Ctypes.(setf x Typs.mjvScene_skinnormal y)

(** get nlight from mjvScene *)
let mjvScene_get_nlight x = Ctypes.(getf x Typs.mjvScene_nlight)

(** set nlight for mjvScene *)
let mjvScene_set_nlight x y = Ctypes.(setf x Typs.mjvScene_nlight y)

(** get lights from mjvScene *)
let mjvScene_get_lights x = Ctypes.(getf x Typs.mjvScene_lights)

(** set lights for mjvScene *)
let mjvScene_set_lights x y = Ctypes.(setf x Typs.mjvScene_lights y)

(** get camera from mjvScene *)
let mjvScene_get_camera x = Ctypes.(getf x Typs.mjvScene_camera)

(** set camera for mjvScene *)
let mjvScene_set_camera x y = Ctypes.(setf x Typs.mjvScene_camera y)

(** get enabletransform from mjvScene *)
let mjvScene_get_enabletransform x = Ctypes.(getf x Typs.mjvScene_enabletransform)

(** set enabletransform for mjvScene *)
let mjvScene_set_enabletransform x y = Ctypes.(setf x Typs.mjvScene_enabletransform y)

(** get translate from mjvScene *)
let mjvScene_get_translate x = Ctypes.(getf x Typs.mjvScene_translate)

(** set translate for mjvScene *)
let mjvScene_set_translate x y = Ctypes.(setf x Typs.mjvScene_translate y)

(** get rotate from mjvScene *)
let mjvScene_get_rotate x = Ctypes.(getf x Typs.mjvScene_rotate)

(** set rotate for mjvScene *)
let mjvScene_set_rotate x y = Ctypes.(setf x Typs.mjvScene_rotate y)

(** get scale from mjvScene *)
let mjvScene_get_scale x = Ctypes.(getf x Typs.mjvScene_scale)

(** set scale for mjvScene *)
let mjvScene_set_scale x y = Ctypes.(setf x Typs.mjvScene_scale y)

(** get stereo from mjvScene *)
let mjvScene_get_stereo x = Ctypes.(getf x Typs.mjvScene_stereo)

(** set stereo for mjvScene *)
let mjvScene_set_stereo x y = Ctypes.(setf x Typs.mjvScene_stereo y)

(** get flags from mjvScene *)
let mjvScene_get_flags x = Ctypes.(getf x Typs.mjvScene_flags)

(** set flags for mjvScene *)
let mjvScene_set_flags x y = Ctypes.(setf x Typs.mjvScene_flags y)

(** get framewidth from mjvScene *)
let mjvScene_get_framewidth x = Ctypes.(getf x Typs.mjvScene_framewidth)

(** set framewidth for mjvScene *)
let mjvScene_set_framewidth x y = Ctypes.(setf x Typs.mjvScene_framewidth y)

(** get framergb from mjvScene *)
let mjvScene_get_framergb x = Ctypes.(getf x Typs.mjvScene_framergb)

(** set framergb for mjvScene *)
let mjvScene_set_framergb x y = Ctypes.(setf x Typs.mjvScene_framergb y)

(** make mjvScene struct *)
let mjvScene_make
    ?mjf_maxgeom
    ?mjf_ngeom
    ?mjf_geoms
    ?mjf_geomorder
    ?mjf_nskin
    ?mjf_skinfacenum
    ?mjf_skinvertadr
    ?mjf_skinvertnum
    ?mjf_skinvert
    ?mjf_skinnormal
    ?mjf_nlight
    ?mjf_lights
    ?mjf_camera
    ?mjf_enabletransform
    ?mjf_translate
    ?mjf_rotate
    ?mjf_scale
    ?mjf_stereo
    ?mjf_flags
    ?mjf_framewidth
    ?mjf_framergb
    ()
  =
  let x = mjvScene_allocate () in
  Option.iter (mjvScene_set_maxgeom x) mjf_maxgeom;
  Option.iter (mjvScene_set_ngeom x) mjf_ngeom;
  Option.iter (mjvScene_set_geoms x) mjf_geoms;
  Option.iter (mjvScene_set_geomorder x) mjf_geomorder;
  Option.iter (mjvScene_set_nskin x) mjf_nskin;
  Option.iter (mjvScene_set_skinfacenum x) mjf_skinfacenum;
  Option.iter (mjvScene_set_skinvertadr x) mjf_skinvertadr;
  Option.iter (mjvScene_set_skinvertnum x) mjf_skinvertnum;
  Option.iter (mjvScene_set_skinvert x) mjf_skinvert;
  Option.iter (mjvScene_set_skinnormal x) mjf_skinnormal;
  Option.iter (mjvScene_set_nlight x) mjf_nlight;
  Option.iter (mjvScene_set_lights x) mjf_lights;
  Option.iter (mjvScene_set_camera x) mjf_camera;
  Option.iter (mjvScene_set_enabletransform x) mjf_enabletransform;
  Option.iter (mjvScene_set_translate x) mjf_translate;
  Option.iter (mjvScene_set_rotate x) mjf_rotate;
  Option.iter (mjvScene_set_scale x) mjf_scale;
  Option.iter (mjvScene_set_stereo x) mjf_stereo;
  Option.iter (mjvScene_set_flags x) mjf_flags;
  Option.iter (mjvScene_set_framewidth x) mjf_framewidth;
  Option.iter (mjvScene_set_framergb x) mjf_framergb;
  x


(** returns mjvScene carray from list of mjvScene *)
let mjvScene_to_carray = Ctypes.CArray.of_list Typs.mjvScene

(** allocate fresh mjvFigure struct *)
let mjvFigure_allocate () = Ctypes.(make Typs.mjvFigure)

(** make null mjvFigure struct ptr *)
let mjvFigure_null () = Ctypes.(from_voidp Typs.mjvFigure null)

(** get flg_legend from mjvFigure *)
let mjvFigure_get_flg_legend x = Ctypes.(getf x Typs.mjvFigure_flg_legend)

(** set flg_legend for mjvFigure *)
let mjvFigure_set_flg_legend x y = Ctypes.(setf x Typs.mjvFigure_flg_legend y)

(** get flg_ticklabel from mjvFigure *)
let mjvFigure_get_flg_ticklabel x = Ctypes.(getf x Typs.mjvFigure_flg_ticklabel)

(** set flg_ticklabel for mjvFigure *)
let mjvFigure_set_flg_ticklabel x y = Ctypes.(setf x Typs.mjvFigure_flg_ticklabel y)

(** get flg_extend from mjvFigure *)
let mjvFigure_get_flg_extend x = Ctypes.(getf x Typs.mjvFigure_flg_extend)

(** set flg_extend for mjvFigure *)
let mjvFigure_set_flg_extend x y = Ctypes.(setf x Typs.mjvFigure_flg_extend y)

(** get flg_barplot from mjvFigure *)
let mjvFigure_get_flg_barplot x = Ctypes.(getf x Typs.mjvFigure_flg_barplot)

(** set flg_barplot for mjvFigure *)
let mjvFigure_set_flg_barplot x y = Ctypes.(setf x Typs.mjvFigure_flg_barplot y)

(** get flg_selection from mjvFigure *)
let mjvFigure_get_flg_selection x = Ctypes.(getf x Typs.mjvFigure_flg_selection)

(** set flg_selection for mjvFigure *)
let mjvFigure_set_flg_selection x y = Ctypes.(setf x Typs.mjvFigure_flg_selection y)

(** get flg_symmetric from mjvFigure *)
let mjvFigure_get_flg_symmetric x = Ctypes.(getf x Typs.mjvFigure_flg_symmetric)

(** set flg_symmetric for mjvFigure *)
let mjvFigure_set_flg_symmetric x y = Ctypes.(setf x Typs.mjvFigure_flg_symmetric y)

(** get linewidth from mjvFigure *)
let mjvFigure_get_linewidth x = Ctypes.(getf x Typs.mjvFigure_linewidth)

(** set linewidth for mjvFigure *)
let mjvFigure_set_linewidth x y = Ctypes.(setf x Typs.mjvFigure_linewidth y)

(** get gridwidth from mjvFigure *)
let mjvFigure_get_gridwidth x = Ctypes.(getf x Typs.mjvFigure_gridwidth)

(** set gridwidth for mjvFigure *)
let mjvFigure_set_gridwidth x y = Ctypes.(setf x Typs.mjvFigure_gridwidth y)

(** get gridsize from mjvFigure *)
let mjvFigure_get_gridsize x = Ctypes.(getf x Typs.mjvFigure_gridsize)

(** set gridsize for mjvFigure *)
let mjvFigure_set_gridsize x y = Ctypes.(setf x Typs.mjvFigure_gridsize y)

(** get gridrgb from mjvFigure *)
let mjvFigure_get_gridrgb x = Ctypes.(getf x Typs.mjvFigure_gridrgb)

(** set gridrgb for mjvFigure *)
let mjvFigure_set_gridrgb x y = Ctypes.(setf x Typs.mjvFigure_gridrgb y)

(** get figurergba from mjvFigure *)
let mjvFigure_get_figurergba x = Ctypes.(getf x Typs.mjvFigure_figurergba)

(** set figurergba for mjvFigure *)
let mjvFigure_set_figurergba x y = Ctypes.(setf x Typs.mjvFigure_figurergba y)

(** get panergba from mjvFigure *)
let mjvFigure_get_panergba x = Ctypes.(getf x Typs.mjvFigure_panergba)

(** set panergba for mjvFigure *)
let mjvFigure_set_panergba x y = Ctypes.(setf x Typs.mjvFigure_panergba y)

(** get legendrgba from mjvFigure *)
let mjvFigure_get_legendrgba x = Ctypes.(getf x Typs.mjvFigure_legendrgba)

(** set legendrgba for mjvFigure *)
let mjvFigure_set_legendrgba x y = Ctypes.(setf x Typs.mjvFigure_legendrgba y)

(** get textrgb from mjvFigure *)
let mjvFigure_get_textrgb x = Ctypes.(getf x Typs.mjvFigure_textrgb)

(** set textrgb for mjvFigure *)
let mjvFigure_set_textrgb x y = Ctypes.(setf x Typs.mjvFigure_textrgb y)

(** get xformat from mjvFigure *)
let mjvFigure_get_xformat x = Ctypes.(getf x Typs.mjvFigure_xformat)

(** set xformat for mjvFigure *)
let mjvFigure_set_xformat x y = Ctypes.(setf x Typs.mjvFigure_xformat y)

(** get yformat from mjvFigure *)
let mjvFigure_get_yformat x = Ctypes.(getf x Typs.mjvFigure_yformat)

(** set yformat for mjvFigure *)
let mjvFigure_set_yformat x y = Ctypes.(setf x Typs.mjvFigure_yformat y)

(** get minwidth from mjvFigure *)
let mjvFigure_get_minwidth x = Ctypes.(getf x Typs.mjvFigure_minwidth)

(** set minwidth for mjvFigure *)
let mjvFigure_set_minwidth x y = Ctypes.(setf x Typs.mjvFigure_minwidth y)

(** get title from mjvFigure *)
let mjvFigure_get_title x = Ctypes.(getf x Typs.mjvFigure_title)

(** set title for mjvFigure *)
let mjvFigure_set_title x y = Ctypes.(setf x Typs.mjvFigure_title y)

(** get xlabel from mjvFigure *)
let mjvFigure_get_xlabel x = Ctypes.(getf x Typs.mjvFigure_xlabel)

(** set xlabel for mjvFigure *)
let mjvFigure_set_xlabel x y = Ctypes.(setf x Typs.mjvFigure_xlabel y)

(** get legendoffset from mjvFigure *)
let mjvFigure_get_legendoffset x = Ctypes.(getf x Typs.mjvFigure_legendoffset)

(** set legendoffset for mjvFigure *)
let mjvFigure_set_legendoffset x y = Ctypes.(setf x Typs.mjvFigure_legendoffset y)

(** get subplot from mjvFigure *)
let mjvFigure_get_subplot x = Ctypes.(getf x Typs.mjvFigure_subplot)

(** set subplot for mjvFigure *)
let mjvFigure_set_subplot x y = Ctypes.(setf x Typs.mjvFigure_subplot y)

(** get highlight from mjvFigure *)
let mjvFigure_get_highlight x = Ctypes.(getf x Typs.mjvFigure_highlight)

(** set highlight for mjvFigure *)
let mjvFigure_set_highlight x y = Ctypes.(setf x Typs.mjvFigure_highlight y)

(** get highlightid from mjvFigure *)
let mjvFigure_get_highlightid x = Ctypes.(getf x Typs.mjvFigure_highlightid)

(** set highlightid for mjvFigure *)
let mjvFigure_set_highlightid x y = Ctypes.(setf x Typs.mjvFigure_highlightid y)

(** get selection from mjvFigure *)
let mjvFigure_get_selection x = Ctypes.(getf x Typs.mjvFigure_selection)

(** set selection for mjvFigure *)
let mjvFigure_set_selection x y = Ctypes.(setf x Typs.mjvFigure_selection y)

(** get linepnt from mjvFigure *)
let mjvFigure_get_linepnt x = Ctypes.(getf x Typs.mjvFigure_linepnt)

(** set linepnt for mjvFigure *)
let mjvFigure_set_linepnt x y = Ctypes.(setf x Typs.mjvFigure_linepnt y)

(** get xaxispixel from mjvFigure *)
let mjvFigure_get_xaxispixel x = Ctypes.(getf x Typs.mjvFigure_xaxispixel)

(** set xaxispixel for mjvFigure *)
let mjvFigure_set_xaxispixel x y = Ctypes.(setf x Typs.mjvFigure_xaxispixel y)

(** get yaxispixel from mjvFigure *)
let mjvFigure_get_yaxispixel x = Ctypes.(getf x Typs.mjvFigure_yaxispixel)

(** set yaxispixel for mjvFigure *)
let mjvFigure_set_yaxispixel x y = Ctypes.(setf x Typs.mjvFigure_yaxispixel y)

(** get xaxisdata from mjvFigure *)
let mjvFigure_get_xaxisdata x = Ctypes.(getf x Typs.mjvFigure_xaxisdata)

(** set xaxisdata for mjvFigure *)
let mjvFigure_set_xaxisdata x y = Ctypes.(setf x Typs.mjvFigure_xaxisdata y)

(** get yaxisdata from mjvFigure *)
let mjvFigure_get_yaxisdata x = Ctypes.(getf x Typs.mjvFigure_yaxisdata)

(** set yaxisdata for mjvFigure *)
let mjvFigure_set_yaxisdata x y = Ctypes.(setf x Typs.mjvFigure_yaxisdata y)

(** make mjvFigure struct *)
let mjvFigure_make
    ?mjf_flg_legend
    ?mjf_flg_ticklabel
    ?mjf_flg_extend
    ?mjf_flg_barplot
    ?mjf_flg_selection
    ?mjf_flg_symmetric
    ?mjf_linewidth
    ?mjf_gridwidth
    ?mjf_gridsize
    ?mjf_gridrgb
    ?mjf_figurergba
    ?mjf_panergba
    ?mjf_legendrgba
    ?mjf_textrgb
    ?mjf_xformat
    ?mjf_yformat
    ?mjf_minwidth
    ?mjf_title
    ?mjf_xlabel
    ?mjf_legendoffset
    ?mjf_subplot
    ?mjf_highlight
    ?mjf_highlightid
    ?mjf_selection
    ?mjf_linepnt
    ?mjf_xaxispixel
    ?mjf_yaxispixel
    ?mjf_xaxisdata
    ?mjf_yaxisdata
    ()
  =
  let x = mjvFigure_allocate () in
  Option.iter (mjvFigure_set_flg_legend x) mjf_flg_legend;
  Option.iter (mjvFigure_set_flg_ticklabel x) mjf_flg_ticklabel;
  Option.iter (mjvFigure_set_flg_extend x) mjf_flg_extend;
  Option.iter (mjvFigure_set_flg_barplot x) mjf_flg_barplot;
  Option.iter (mjvFigure_set_flg_selection x) mjf_flg_selection;
  Option.iter (mjvFigure_set_flg_symmetric x) mjf_flg_symmetric;
  Option.iter (mjvFigure_set_linewidth x) mjf_linewidth;
  Option.iter (mjvFigure_set_gridwidth x) mjf_gridwidth;
  Option.iter (mjvFigure_set_gridsize x) mjf_gridsize;
  Option.iter (mjvFigure_set_gridrgb x) mjf_gridrgb;
  Option.iter (mjvFigure_set_figurergba x) mjf_figurergba;
  Option.iter (mjvFigure_set_panergba x) mjf_panergba;
  Option.iter (mjvFigure_set_legendrgba x) mjf_legendrgba;
  Option.iter (mjvFigure_set_textrgb x) mjf_textrgb;
  Option.iter (mjvFigure_set_xformat x) mjf_xformat;
  Option.iter (mjvFigure_set_yformat x) mjf_yformat;
  Option.iter (mjvFigure_set_minwidth x) mjf_minwidth;
  Option.iter (mjvFigure_set_title x) mjf_title;
  Option.iter (mjvFigure_set_xlabel x) mjf_xlabel;
  Option.iter (mjvFigure_set_legendoffset x) mjf_legendoffset;
  Option.iter (mjvFigure_set_subplot x) mjf_subplot;
  Option.iter (mjvFigure_set_highlight x) mjf_highlight;
  Option.iter (mjvFigure_set_highlightid x) mjf_highlightid;
  Option.iter (mjvFigure_set_selection x) mjf_selection;
  Option.iter (mjvFigure_set_linepnt x) mjf_linepnt;
  Option.iter (mjvFigure_set_xaxispixel x) mjf_xaxispixel;
  Option.iter (mjvFigure_set_yaxispixel x) mjf_yaxispixel;
  Option.iter (mjvFigure_set_xaxisdata x) mjf_xaxisdata;
  Option.iter (mjvFigure_set_yaxisdata x) mjf_yaxisdata;
  x


(** returns mjvFigure carray from list of mjvFigure *)
let mjvFigure_to_carray = Ctypes.CArray.of_list Typs.mjvFigure

(** convert mjtGridPos type to int *)
let mjtGridPos_to_int = function
  | MjGRID_TOPLEFT     -> Typs.mjGRID_TOPLEFT |> Int64.to_int
  | MjGRID_TOPRIGHT    -> Typs.mjGRID_TOPRIGHT |> Int64.to_int
  | MjGRID_BOTTOMLEFT  -> Typs.mjGRID_BOTTOMLEFT |> Int64.to_int
  | MjGRID_BOTTOMRIGHT -> Typs.mjGRID_BOTTOMRIGHT |> Int64.to_int


(** convert mjtFramebuffer type to int *)
let mjtFramebuffer_to_int = function
  | MjFB_WINDOW    -> Typs.mjFB_WINDOW |> Int64.to_int
  | MjFB_OFFSCREEN -> Typs.mjFB_OFFSCREEN |> Int64.to_int


(** convert mjtFontScale type to int *)
let mjtFontScale_to_int = function
  | MjFONTSCALE_50  -> Typs.mjFONTSCALE_50 |> Int64.to_int
  | MjFONTSCALE_100 -> Typs.mjFONTSCALE_100 |> Int64.to_int
  | MjFONTSCALE_150 -> Typs.mjFONTSCALE_150 |> Int64.to_int
  | MjFONTSCALE_200 -> Typs.mjFONTSCALE_200 |> Int64.to_int
  | MjFONTSCALE_250 -> Typs.mjFONTSCALE_250 |> Int64.to_int
  | MjFONTSCALE_300 -> Typs.mjFONTSCALE_300 |> Int64.to_int


(** convert mjtFont type to int *)
let mjtFont_to_int = function
  | MjFONT_NORMAL -> Typs.mjFONT_NORMAL |> Int64.to_int
  | MjFONT_SHADOW -> Typs.mjFONT_SHADOW |> Int64.to_int
  | MjFONT_BIG    -> Typs.mjFONT_BIG |> Int64.to_int


(** allocate fresh mjrRect struct *)
let mjrRect_allocate () = Ctypes.(make Typs.mjrRect)

(** make null mjrRect struct ptr *)
let mjrRect_null () = Ctypes.(from_voidp Typs.mjrRect null)

(** get left from mjrRect *)
let mjrRect_get_left x = Ctypes.(getf x Typs.mjrRect_left)

(** set left for mjrRect *)
let mjrRect_set_left x y = Ctypes.(setf x Typs.mjrRect_left y)

(** get bottom from mjrRect *)
let mjrRect_get_bottom x = Ctypes.(getf x Typs.mjrRect_bottom)

(** set bottom for mjrRect *)
let mjrRect_set_bottom x y = Ctypes.(setf x Typs.mjrRect_bottom y)

(** get width from mjrRect *)
let mjrRect_get_width x = Ctypes.(getf x Typs.mjrRect_width)

(** set width for mjrRect *)
let mjrRect_set_width x y = Ctypes.(setf x Typs.mjrRect_width y)

(** get height from mjrRect *)
let mjrRect_get_height x = Ctypes.(getf x Typs.mjrRect_height)

(** set height for mjrRect *)
let mjrRect_set_height x y = Ctypes.(setf x Typs.mjrRect_height y)

(** make mjrRect struct *)
let mjrRect_make ?mjf_left ?mjf_bottom ?mjf_width ?mjf_height () =
  let x = mjrRect_allocate () in
  Option.iter (mjrRect_set_left x) mjf_left;
  Option.iter (mjrRect_set_bottom x) mjf_bottom;
  Option.iter (mjrRect_set_width x) mjf_width;
  Option.iter (mjrRect_set_height x) mjf_height;
  x


(** returns mjrRect carray from list of mjrRect *)
let mjrRect_to_carray = Ctypes.CArray.of_list Typs.mjrRect

(** allocate fresh mjrContext struct *)
let mjrContext_allocate () = Ctypes.(make Typs.mjrContext)

(** make null mjrContext struct ptr *)
let mjrContext_null () = Ctypes.(from_voidp Typs.mjrContext null)

(** get lineWidth from mjrContext *)
let mjrContext_get_lineWidth x = Ctypes.(getf x Typs.mjrContext_lineWidth)

(** set lineWidth for mjrContext *)
let mjrContext_set_lineWidth x y = Ctypes.(setf x Typs.mjrContext_lineWidth y)

(** get shadowClip from mjrContext *)
let mjrContext_get_shadowClip x = Ctypes.(getf x Typs.mjrContext_shadowClip)

(** set shadowClip for mjrContext *)
let mjrContext_set_shadowClip x y = Ctypes.(setf x Typs.mjrContext_shadowClip y)

(** get shadowScale from mjrContext *)
let mjrContext_get_shadowScale x = Ctypes.(getf x Typs.mjrContext_shadowScale)

(** set shadowScale for mjrContext *)
let mjrContext_set_shadowScale x y = Ctypes.(setf x Typs.mjrContext_shadowScale y)

(** get fogStart from mjrContext *)
let mjrContext_get_fogStart x = Ctypes.(getf x Typs.mjrContext_fogStart)

(** set fogStart for mjrContext *)
let mjrContext_set_fogStart x y = Ctypes.(setf x Typs.mjrContext_fogStart y)

(** get fogEnd from mjrContext *)
let mjrContext_get_fogEnd x = Ctypes.(getf x Typs.mjrContext_fogEnd)

(** set fogEnd for mjrContext *)
let mjrContext_set_fogEnd x y = Ctypes.(setf x Typs.mjrContext_fogEnd y)

(** get fogRGBA from mjrContext *)
let mjrContext_get_fogRGBA x = Ctypes.(getf x Typs.mjrContext_fogRGBA)

(** set fogRGBA for mjrContext *)
let mjrContext_set_fogRGBA x y = Ctypes.(setf x Typs.mjrContext_fogRGBA y)

(** get shadowSize from mjrContext *)
let mjrContext_get_shadowSize x = Ctypes.(getf x Typs.mjrContext_shadowSize)

(** set shadowSize for mjrContext *)
let mjrContext_set_shadowSize x y = Ctypes.(setf x Typs.mjrContext_shadowSize y)

(** get offWidth from mjrContext *)
let mjrContext_get_offWidth x = Ctypes.(getf x Typs.mjrContext_offWidth)

(** set offWidth for mjrContext *)
let mjrContext_set_offWidth x y = Ctypes.(setf x Typs.mjrContext_offWidth y)

(** get offHeight from mjrContext *)
let mjrContext_get_offHeight x = Ctypes.(getf x Typs.mjrContext_offHeight)

(** set offHeight for mjrContext *)
let mjrContext_set_offHeight x y = Ctypes.(setf x Typs.mjrContext_offHeight y)

(** get offSamples from mjrContext *)
let mjrContext_get_offSamples x = Ctypes.(getf x Typs.mjrContext_offSamples)

(** set offSamples for mjrContext *)
let mjrContext_set_offSamples x y = Ctypes.(setf x Typs.mjrContext_offSamples y)

(** get fontScale from mjrContext *)
let mjrContext_get_fontScale x = Ctypes.(getf x Typs.mjrContext_fontScale)

(** set fontScale for mjrContext *)
let mjrContext_set_fontScale x y = Ctypes.(setf x Typs.mjrContext_fontScale y)

(** get auxWidth from mjrContext *)
let mjrContext_get_auxWidth x = Ctypes.(getf x Typs.mjrContext_auxWidth)

(** set auxWidth for mjrContext *)
let mjrContext_set_auxWidth x y = Ctypes.(setf x Typs.mjrContext_auxWidth y)

(** get auxHeight from mjrContext *)
let mjrContext_get_auxHeight x = Ctypes.(getf x Typs.mjrContext_auxHeight)

(** set auxHeight for mjrContext *)
let mjrContext_set_auxHeight x y = Ctypes.(setf x Typs.mjrContext_auxHeight y)

(** get auxSamples from mjrContext *)
let mjrContext_get_auxSamples x = Ctypes.(getf x Typs.mjrContext_auxSamples)

(** set auxSamples for mjrContext *)
let mjrContext_set_auxSamples x y = Ctypes.(setf x Typs.mjrContext_auxSamples y)

(** get ntexture from mjrContext *)
let mjrContext_get_ntexture x = Ctypes.(getf x Typs.mjrContext_ntexture)

(** set ntexture for mjrContext *)
let mjrContext_set_ntexture x y = Ctypes.(setf x Typs.mjrContext_ntexture y)

(** get textureType from mjrContext *)
let mjrContext_get_textureType x = Ctypes.(getf x Typs.mjrContext_textureType)

(** set textureType for mjrContext *)
let mjrContext_set_textureType x y = Ctypes.(setf x Typs.mjrContext_textureType y)

(** get rangePlane from mjrContext *)
let mjrContext_get_rangePlane x = Ctypes.(getf x Typs.mjrContext_rangePlane)

(** set rangePlane for mjrContext *)
let mjrContext_set_rangePlane x y = Ctypes.(setf x Typs.mjrContext_rangePlane y)

(** get rangeMesh from mjrContext *)
let mjrContext_get_rangeMesh x = Ctypes.(getf x Typs.mjrContext_rangeMesh)

(** set rangeMesh for mjrContext *)
let mjrContext_set_rangeMesh x y = Ctypes.(setf x Typs.mjrContext_rangeMesh y)

(** get rangeHField from mjrContext *)
let mjrContext_get_rangeHField x = Ctypes.(getf x Typs.mjrContext_rangeHField)

(** set rangeHField for mjrContext *)
let mjrContext_set_rangeHField x y = Ctypes.(setf x Typs.mjrContext_rangeHField y)

(** get rangeBuiltin from mjrContext *)
let mjrContext_get_rangeBuiltin x = Ctypes.(getf x Typs.mjrContext_rangeBuiltin)

(** set rangeBuiltin for mjrContext *)
let mjrContext_set_rangeBuiltin x y = Ctypes.(setf x Typs.mjrContext_rangeBuiltin y)

(** get rangeFont from mjrContext *)
let mjrContext_get_rangeFont x = Ctypes.(getf x Typs.mjrContext_rangeFont)

(** set rangeFont for mjrContext *)
let mjrContext_set_rangeFont x y = Ctypes.(setf x Typs.mjrContext_rangeFont y)

(** get nskin from mjrContext *)
let mjrContext_get_nskin x = Ctypes.(getf x Typs.mjrContext_nskin)

(** set nskin for mjrContext *)
let mjrContext_set_nskin x y = Ctypes.(setf x Typs.mjrContext_nskin y)

(** get charWidth from mjrContext *)
let mjrContext_get_charWidth x = Ctypes.(getf x Typs.mjrContext_charWidth)

(** set charWidth for mjrContext *)
let mjrContext_set_charWidth x y = Ctypes.(setf x Typs.mjrContext_charWidth y)

(** get charWidthBig from mjrContext *)
let mjrContext_get_charWidthBig x = Ctypes.(getf x Typs.mjrContext_charWidthBig)

(** set charWidthBig for mjrContext *)
let mjrContext_set_charWidthBig x y = Ctypes.(setf x Typs.mjrContext_charWidthBig y)

(** get charHeight from mjrContext *)
let mjrContext_get_charHeight x = Ctypes.(getf x Typs.mjrContext_charHeight)

(** set charHeight for mjrContext *)
let mjrContext_set_charHeight x y = Ctypes.(setf x Typs.mjrContext_charHeight y)

(** get charHeightBig from mjrContext *)
let mjrContext_get_charHeightBig x = Ctypes.(getf x Typs.mjrContext_charHeightBig)

(** set charHeightBig for mjrContext *)
let mjrContext_set_charHeightBig x y = Ctypes.(setf x Typs.mjrContext_charHeightBig y)

(** get glewInitialized from mjrContext *)
let mjrContext_get_glewInitialized x = Ctypes.(getf x Typs.mjrContext_glewInitialized)

(** set glewInitialized for mjrContext *)
let mjrContext_set_glewInitialized x y = Ctypes.(setf x Typs.mjrContext_glewInitialized y)

(** get windowAvailable from mjrContext *)
let mjrContext_get_windowAvailable x = Ctypes.(getf x Typs.mjrContext_windowAvailable)

(** set windowAvailable for mjrContext *)
let mjrContext_set_windowAvailable x y = Ctypes.(setf x Typs.mjrContext_windowAvailable y)

(** get windowSamples from mjrContext *)
let mjrContext_get_windowSamples x = Ctypes.(getf x Typs.mjrContext_windowSamples)

(** set windowSamples for mjrContext *)
let mjrContext_set_windowSamples x y = Ctypes.(setf x Typs.mjrContext_windowSamples y)

(** get windowStereo from mjrContext *)
let mjrContext_get_windowStereo x = Ctypes.(getf x Typs.mjrContext_windowStereo)

(** set windowStereo for mjrContext *)
let mjrContext_set_windowStereo x y = Ctypes.(setf x Typs.mjrContext_windowStereo y)

(** get windowDoublebuffer from mjrContext *)
let mjrContext_get_windowDoublebuffer x =
  Ctypes.(getf x Typs.mjrContext_windowDoublebuffer)


(** set windowDoublebuffer for mjrContext *)
let mjrContext_set_windowDoublebuffer x y =
  Ctypes.(setf x Typs.mjrContext_windowDoublebuffer y)


(** get currentBuffer from mjrContext *)
let mjrContext_get_currentBuffer x = Ctypes.(getf x Typs.mjrContext_currentBuffer)

(** set currentBuffer for mjrContext *)
let mjrContext_set_currentBuffer x y = Ctypes.(setf x Typs.mjrContext_currentBuffer y)

(** make mjrContext struct *)
let mjrContext_make
    ?mjf_lineWidth
    ?mjf_shadowClip
    ?mjf_shadowScale
    ?mjf_fogStart
    ?mjf_fogEnd
    ?mjf_fogRGBA
    ?mjf_shadowSize
    ?mjf_offWidth
    ?mjf_offHeight
    ?mjf_offSamples
    ?mjf_fontScale
    ?mjf_auxWidth
    ?mjf_auxHeight
    ?mjf_auxSamples
    ?mjf_ntexture
    ?mjf_textureType
    ?mjf_rangePlane
    ?mjf_rangeMesh
    ?mjf_rangeHField
    ?mjf_rangeBuiltin
    ?mjf_rangeFont
    ?mjf_nskin
    ?mjf_charWidth
    ?mjf_charWidthBig
    ?mjf_charHeight
    ?mjf_charHeightBig
    ?mjf_glewInitialized
    ?mjf_windowAvailable
    ?mjf_windowSamples
    ?mjf_windowStereo
    ?mjf_windowDoublebuffer
    ?mjf_currentBuffer
    ()
  =
  let x = mjrContext_allocate () in
  Option.iter (mjrContext_set_lineWidth x) mjf_lineWidth;
  Option.iter (mjrContext_set_shadowClip x) mjf_shadowClip;
  Option.iter (mjrContext_set_shadowScale x) mjf_shadowScale;
  Option.iter (mjrContext_set_fogStart x) mjf_fogStart;
  Option.iter (mjrContext_set_fogEnd x) mjf_fogEnd;
  Option.iter (mjrContext_set_fogRGBA x) mjf_fogRGBA;
  Option.iter (mjrContext_set_shadowSize x) mjf_shadowSize;
  Option.iter (mjrContext_set_offWidth x) mjf_offWidth;
  Option.iter (mjrContext_set_offHeight x) mjf_offHeight;
  Option.iter (mjrContext_set_offSamples x) mjf_offSamples;
  Option.iter (mjrContext_set_fontScale x) mjf_fontScale;
  Option.iter (mjrContext_set_auxWidth x) mjf_auxWidth;
  Option.iter (mjrContext_set_auxHeight x) mjf_auxHeight;
  Option.iter (mjrContext_set_auxSamples x) mjf_auxSamples;
  Option.iter (mjrContext_set_ntexture x) mjf_ntexture;
  Option.iter (mjrContext_set_textureType x) mjf_textureType;
  Option.iter (mjrContext_set_rangePlane x) mjf_rangePlane;
  Option.iter (mjrContext_set_rangeMesh x) mjf_rangeMesh;
  Option.iter (mjrContext_set_rangeHField x) mjf_rangeHField;
  Option.iter (mjrContext_set_rangeBuiltin x) mjf_rangeBuiltin;
  Option.iter (mjrContext_set_rangeFont x) mjf_rangeFont;
  Option.iter (mjrContext_set_nskin x) mjf_nskin;
  Option.iter (mjrContext_set_charWidth x) mjf_charWidth;
  Option.iter (mjrContext_set_charWidthBig x) mjf_charWidthBig;
  Option.iter (mjrContext_set_charHeight x) mjf_charHeight;
  Option.iter (mjrContext_set_charHeightBig x) mjf_charHeightBig;
  Option.iter (mjrContext_set_glewInitialized x) mjf_glewInitialized;
  Option.iter (mjrContext_set_windowAvailable x) mjf_windowAvailable;
  Option.iter (mjrContext_set_windowSamples x) mjf_windowSamples;
  Option.iter (mjrContext_set_windowStereo x) mjf_windowStereo;
  Option.iter (mjrContext_set_windowDoublebuffer x) mjf_windowDoublebuffer;
  Option.iter (mjrContext_set_currentBuffer x) mjf_currentBuffer;
  x


(** returns mjrContext carray from list of mjrContext *)
let mjrContext_to_carray = Ctypes.CArray.of_list Typs.mjrContext

(** convert mjtButton type to int *)
let mjtButton_to_int = function
  | MjBUTTON_NONE   -> Typs.mjBUTTON_NONE |> Int64.to_int
  | MjBUTTON_LEFT   -> Typs.mjBUTTON_LEFT |> Int64.to_int
  | MjBUTTON_RIGHT  -> Typs.mjBUTTON_RIGHT |> Int64.to_int
  | MjBUTTON_MIDDLE -> Typs.mjBUTTON_MIDDLE |> Int64.to_int


(** convert mjtEvent type to int *)
let mjtEvent_to_int = function
  | MjEVENT_NONE    -> Typs.mjEVENT_NONE |> Int64.to_int
  | MjEVENT_MOVE    -> Typs.mjEVENT_MOVE |> Int64.to_int
  | MjEVENT_PRESS   -> Typs.mjEVENT_PRESS |> Int64.to_int
  | MjEVENT_RELEASE -> Typs.mjEVENT_RELEASE |> Int64.to_int
  | MjEVENT_SCROLL  -> Typs.mjEVENT_SCROLL |> Int64.to_int
  | MjEVENT_KEY     -> Typs.mjEVENT_KEY |> Int64.to_int
  | MjEVENT_RESIZE  -> Typs.mjEVENT_RESIZE |> Int64.to_int


(** convert mjtItem type to int *)
let mjtItem_to_int = function
  | MjITEM_END       -> Typs.mjITEM_END |> Int64.to_int
  | MjITEM_SECTION   -> Typs.mjITEM_SECTION |> Int64.to_int
  | MjITEM_SEPARATOR -> Typs.mjITEM_SEPARATOR |> Int64.to_int
  | MjITEM_STATIC    -> Typs.mjITEM_STATIC |> Int64.to_int
  | MjITEM_BUTTON    -> Typs.mjITEM_BUTTON |> Int64.to_int
  | MjITEM_CHECKINT  -> Typs.mjITEM_CHECKINT |> Int64.to_int
  | MjITEM_CHECKBYTE -> Typs.mjITEM_CHECKBYTE |> Int64.to_int
  | MjITEM_RADIO     -> Typs.mjITEM_RADIO |> Int64.to_int
  | MjITEM_RADIOLINE -> Typs.mjITEM_RADIOLINE |> Int64.to_int
  | MjITEM_SELECT    -> Typs.mjITEM_SELECT |> Int64.to_int
  | MjITEM_SLIDERINT -> Typs.mjITEM_SLIDERINT |> Int64.to_int
  | MjITEM_SLIDERNUM -> Typs.mjITEM_SLIDERNUM |> Int64.to_int
  | MjITEM_EDITINT   -> Typs.mjITEM_EDITINT |> Int64.to_int
  | MjITEM_EDITNUM   -> Typs.mjITEM_EDITNUM |> Int64.to_int
  | MjITEM_EDITTXT   -> Typs.mjITEM_EDITTXT |> Int64.to_int
  | MjNITEM          -> Typs.mjNITEM |> Int64.to_int


(** allocate fresh mjuiState struct *)
let mjuiState_allocate () = Ctypes.(make Typs.mjuiState)

(** make null mjuiState struct ptr *)
let mjuiState_null () = Ctypes.(from_voidp Typs.mjuiState null)

(** get nrect from mjuiState *)
let mjuiState_get_nrect x = Ctypes.(getf x Typs.mjuiState_nrect)

(** set nrect for mjuiState *)
let mjuiState_set_nrect x y = Ctypes.(setf x Typs.mjuiState_nrect y)

(** get rect from mjuiState *)
let mjuiState_get_rect x = Ctypes.(getf x Typs.mjuiState_rect)

(** set rect for mjuiState *)
let mjuiState_set_rect x y = Ctypes.(setf x Typs.mjuiState_rect y)

(** get userdata from mjuiState *)
let mjuiState_get_userdata x = Ctypes.(getf x Typs.mjuiState_userdata)

(** set userdata for mjuiState *)
let mjuiState_set_userdata x y = Ctypes.(setf x Typs.mjuiState_userdata y)

(** get type from mjuiState *)
let mjuiState_get_type x = Ctypes.(getf x Typs.mjuiState_type)

(** set type for mjuiState *)
let mjuiState_set_type x y = Ctypes.(setf x Typs.mjuiState_type y)

(** get left from mjuiState *)
let mjuiState_get_left x = Ctypes.(getf x Typs.mjuiState_left)

(** set left for mjuiState *)
let mjuiState_set_left x y = Ctypes.(setf x Typs.mjuiState_left y)

(** get right from mjuiState *)
let mjuiState_get_right x = Ctypes.(getf x Typs.mjuiState_right)

(** set right for mjuiState *)
let mjuiState_set_right x y = Ctypes.(setf x Typs.mjuiState_right y)

(** get middle from mjuiState *)
let mjuiState_get_middle x = Ctypes.(getf x Typs.mjuiState_middle)

(** set middle for mjuiState *)
let mjuiState_set_middle x y = Ctypes.(setf x Typs.mjuiState_middle y)

(** get doubleclick from mjuiState *)
let mjuiState_get_doubleclick x = Ctypes.(getf x Typs.mjuiState_doubleclick)

(** set doubleclick for mjuiState *)
let mjuiState_set_doubleclick x y = Ctypes.(setf x Typs.mjuiState_doubleclick y)

(** get button from mjuiState *)
let mjuiState_get_button x = Ctypes.(getf x Typs.mjuiState_button)

(** set button for mjuiState *)
let mjuiState_set_button x y = Ctypes.(setf x Typs.mjuiState_button y)

(** get buttontime from mjuiState *)
let mjuiState_get_buttontime x = Ctypes.(getf x Typs.mjuiState_buttontime)

(** set buttontime for mjuiState *)
let mjuiState_set_buttontime x y = Ctypes.(setf x Typs.mjuiState_buttontime y)

(** get x from mjuiState *)
let mjuiState_get_x x = Ctypes.(getf x Typs.mjuiState_x)

(** set x for mjuiState *)
let mjuiState_set_x x y = Ctypes.(setf x Typs.mjuiState_x y)

(** get y from mjuiState *)
let mjuiState_get_y x = Ctypes.(getf x Typs.mjuiState_y)

(** set y for mjuiState *)
let mjuiState_set_y x y = Ctypes.(setf x Typs.mjuiState_y y)

(** get dx from mjuiState *)
let mjuiState_get_dx x = Ctypes.(getf x Typs.mjuiState_dx)

(** set dx for mjuiState *)
let mjuiState_set_dx x y = Ctypes.(setf x Typs.mjuiState_dx y)

(** get dy from mjuiState *)
let mjuiState_get_dy x = Ctypes.(getf x Typs.mjuiState_dy)

(** set dy for mjuiState *)
let mjuiState_set_dy x y = Ctypes.(setf x Typs.mjuiState_dy y)

(** get sx from mjuiState *)
let mjuiState_get_sx x = Ctypes.(getf x Typs.mjuiState_sx)

(** set sx for mjuiState *)
let mjuiState_set_sx x y = Ctypes.(setf x Typs.mjuiState_sx y)

(** get sy from mjuiState *)
let mjuiState_get_sy x = Ctypes.(getf x Typs.mjuiState_sy)

(** set sy for mjuiState *)
let mjuiState_set_sy x y = Ctypes.(setf x Typs.mjuiState_sy y)

(** get control from mjuiState *)
let mjuiState_get_control x = Ctypes.(getf x Typs.mjuiState_control)

(** set control for mjuiState *)
let mjuiState_set_control x y = Ctypes.(setf x Typs.mjuiState_control y)

(** get shift from mjuiState *)
let mjuiState_get_shift x = Ctypes.(getf x Typs.mjuiState_shift)

(** set shift for mjuiState *)
let mjuiState_set_shift x y = Ctypes.(setf x Typs.mjuiState_shift y)

(** get alt from mjuiState *)
let mjuiState_get_alt x = Ctypes.(getf x Typs.mjuiState_alt)

(** set alt for mjuiState *)
let mjuiState_set_alt x y = Ctypes.(setf x Typs.mjuiState_alt y)

(** get key from mjuiState *)
let mjuiState_get_key x = Ctypes.(getf x Typs.mjuiState_key)

(** set key for mjuiState *)
let mjuiState_set_key x y = Ctypes.(setf x Typs.mjuiState_key y)

(** get keytime from mjuiState *)
let mjuiState_get_keytime x = Ctypes.(getf x Typs.mjuiState_keytime)

(** set keytime for mjuiState *)
let mjuiState_set_keytime x y = Ctypes.(setf x Typs.mjuiState_keytime y)

(** get mouserect from mjuiState *)
let mjuiState_get_mouserect x = Ctypes.(getf x Typs.mjuiState_mouserect)

(** set mouserect for mjuiState *)
let mjuiState_set_mouserect x y = Ctypes.(setf x Typs.mjuiState_mouserect y)

(** get dragrect from mjuiState *)
let mjuiState_get_dragrect x = Ctypes.(getf x Typs.mjuiState_dragrect)

(** set dragrect for mjuiState *)
let mjuiState_set_dragrect x y = Ctypes.(setf x Typs.mjuiState_dragrect y)

(** get dragbutton from mjuiState *)
let mjuiState_get_dragbutton x = Ctypes.(getf x Typs.mjuiState_dragbutton)

(** set dragbutton for mjuiState *)
let mjuiState_set_dragbutton x y = Ctypes.(setf x Typs.mjuiState_dragbutton y)

(** make mjuiState struct *)
let mjuiState_make
    ?mjf_nrect
    ?mjf_rect
    ?mjf_userdata
    ?mjf_type
    ?mjf_left
    ?mjf_right
    ?mjf_middle
    ?mjf_doubleclick
    ?mjf_button
    ?mjf_buttontime
    ?mjf_x
    ?mjf_y
    ?mjf_dx
    ?mjf_dy
    ?mjf_sx
    ?mjf_sy
    ?mjf_control
    ?mjf_shift
    ?mjf_alt
    ?mjf_key
    ?mjf_keytime
    ?mjf_mouserect
    ?mjf_dragrect
    ?mjf_dragbutton
    ()
  =
  let x = mjuiState_allocate () in
  Option.iter (mjuiState_set_nrect x) mjf_nrect;
  Option.iter (mjuiState_set_rect x) mjf_rect;
  Option.iter (mjuiState_set_userdata x) mjf_userdata;
  Option.iter (mjuiState_set_type x) mjf_type;
  Option.iter (mjuiState_set_left x) mjf_left;
  Option.iter (mjuiState_set_right x) mjf_right;
  Option.iter (mjuiState_set_middle x) mjf_middle;
  Option.iter (mjuiState_set_doubleclick x) mjf_doubleclick;
  Option.iter (mjuiState_set_button x) mjf_button;
  Option.iter (mjuiState_set_buttontime x) mjf_buttontime;
  Option.iter (mjuiState_set_x x) mjf_x;
  Option.iter (mjuiState_set_y x) mjf_y;
  Option.iter (mjuiState_set_dx x) mjf_dx;
  Option.iter (mjuiState_set_dy x) mjf_dy;
  Option.iter (mjuiState_set_sx x) mjf_sx;
  Option.iter (mjuiState_set_sy x) mjf_sy;
  Option.iter (mjuiState_set_control x) mjf_control;
  Option.iter (mjuiState_set_shift x) mjf_shift;
  Option.iter (mjuiState_set_alt x) mjf_alt;
  Option.iter (mjuiState_set_key x) mjf_key;
  Option.iter (mjuiState_set_keytime x) mjf_keytime;
  Option.iter (mjuiState_set_mouserect x) mjf_mouserect;
  Option.iter (mjuiState_set_dragrect x) mjf_dragrect;
  Option.iter (mjuiState_set_dragbutton x) mjf_dragbutton;
  x


(** returns mjuiState carray from list of mjuiState *)
let mjuiState_to_carray = Ctypes.CArray.of_list Typs.mjuiState

(** allocate fresh mjuiThemeSpacing struct *)
let mjuiThemeSpacing_allocate () = Ctypes.(make Typs.mjuiThemeSpacing)

(** make null mjuiThemeSpacing struct ptr *)
let mjuiThemeSpacing_null () = Ctypes.(from_voidp Typs.mjuiThemeSpacing null)

(** get total from mjuiThemeSpacing *)
let mjuiThemeSpacing_get_total x = Ctypes.(getf x Typs.mjuiThemeSpacing_total)

(** set total for mjuiThemeSpacing *)
let mjuiThemeSpacing_set_total x y = Ctypes.(setf x Typs.mjuiThemeSpacing_total y)

(** get scroll from mjuiThemeSpacing *)
let mjuiThemeSpacing_get_scroll x = Ctypes.(getf x Typs.mjuiThemeSpacing_scroll)

(** set scroll for mjuiThemeSpacing *)
let mjuiThemeSpacing_set_scroll x y = Ctypes.(setf x Typs.mjuiThemeSpacing_scroll y)

(** get label from mjuiThemeSpacing *)
let mjuiThemeSpacing_get_label x = Ctypes.(getf x Typs.mjuiThemeSpacing_label)

(** set label for mjuiThemeSpacing *)
let mjuiThemeSpacing_set_label x y = Ctypes.(setf x Typs.mjuiThemeSpacing_label y)

(** get section from mjuiThemeSpacing *)
let mjuiThemeSpacing_get_section x = Ctypes.(getf x Typs.mjuiThemeSpacing_section)

(** set section for mjuiThemeSpacing *)
let mjuiThemeSpacing_set_section x y = Ctypes.(setf x Typs.mjuiThemeSpacing_section y)

(** get itemside from mjuiThemeSpacing *)
let mjuiThemeSpacing_get_itemside x = Ctypes.(getf x Typs.mjuiThemeSpacing_itemside)

(** set itemside for mjuiThemeSpacing *)
let mjuiThemeSpacing_set_itemside x y = Ctypes.(setf x Typs.mjuiThemeSpacing_itemside y)

(** get itemmid from mjuiThemeSpacing *)
let mjuiThemeSpacing_get_itemmid x = Ctypes.(getf x Typs.mjuiThemeSpacing_itemmid)

(** set itemmid for mjuiThemeSpacing *)
let mjuiThemeSpacing_set_itemmid x y = Ctypes.(setf x Typs.mjuiThemeSpacing_itemmid y)

(** get itemver from mjuiThemeSpacing *)
let mjuiThemeSpacing_get_itemver x = Ctypes.(getf x Typs.mjuiThemeSpacing_itemver)

(** set itemver for mjuiThemeSpacing *)
let mjuiThemeSpacing_set_itemver x y = Ctypes.(setf x Typs.mjuiThemeSpacing_itemver y)

(** get texthor from mjuiThemeSpacing *)
let mjuiThemeSpacing_get_texthor x = Ctypes.(getf x Typs.mjuiThemeSpacing_texthor)

(** set texthor for mjuiThemeSpacing *)
let mjuiThemeSpacing_set_texthor x y = Ctypes.(setf x Typs.mjuiThemeSpacing_texthor y)

(** get textver from mjuiThemeSpacing *)
let mjuiThemeSpacing_get_textver x = Ctypes.(getf x Typs.mjuiThemeSpacing_textver)

(** set textver for mjuiThemeSpacing *)
let mjuiThemeSpacing_set_textver x y = Ctypes.(setf x Typs.mjuiThemeSpacing_textver y)

(** get linescroll from mjuiThemeSpacing *)
let mjuiThemeSpacing_get_linescroll x = Ctypes.(getf x Typs.mjuiThemeSpacing_linescroll)

(** set linescroll for mjuiThemeSpacing *)
let mjuiThemeSpacing_set_linescroll x y =
  Ctypes.(setf x Typs.mjuiThemeSpacing_linescroll y)


(** get samples from mjuiThemeSpacing *)
let mjuiThemeSpacing_get_samples x = Ctypes.(getf x Typs.mjuiThemeSpacing_samples)

(** set samples for mjuiThemeSpacing *)
let mjuiThemeSpacing_set_samples x y = Ctypes.(setf x Typs.mjuiThemeSpacing_samples y)

(** make mjuiThemeSpacing struct *)
let mjuiThemeSpacing_make
    ?mjf_total
    ?mjf_scroll
    ?mjf_label
    ?mjf_section
    ?mjf_itemside
    ?mjf_itemmid
    ?mjf_itemver
    ?mjf_texthor
    ?mjf_textver
    ?mjf_linescroll
    ?mjf_samples
    ()
  =
  let x = mjuiThemeSpacing_allocate () in
  Option.iter (mjuiThemeSpacing_set_total x) mjf_total;
  Option.iter (mjuiThemeSpacing_set_scroll x) mjf_scroll;
  Option.iter (mjuiThemeSpacing_set_label x) mjf_label;
  Option.iter (mjuiThemeSpacing_set_section x) mjf_section;
  Option.iter (mjuiThemeSpacing_set_itemside x) mjf_itemside;
  Option.iter (mjuiThemeSpacing_set_itemmid x) mjf_itemmid;
  Option.iter (mjuiThemeSpacing_set_itemver x) mjf_itemver;
  Option.iter (mjuiThemeSpacing_set_texthor x) mjf_texthor;
  Option.iter (mjuiThemeSpacing_set_textver x) mjf_textver;
  Option.iter (mjuiThemeSpacing_set_linescroll x) mjf_linescroll;
  Option.iter (mjuiThemeSpacing_set_samples x) mjf_samples;
  x


(** returns mjuiThemeSpacing carray from list of mjuiThemeSpacing *)
let mjuiThemeSpacing_to_carray = Ctypes.CArray.of_list Typs.mjuiThemeSpacing

(** allocate fresh mjuiThemeColor struct *)
let mjuiThemeColor_allocate () = Ctypes.(make Typs.mjuiThemeColor)

(** make null mjuiThemeColor struct ptr *)
let mjuiThemeColor_null () = Ctypes.(from_voidp Typs.mjuiThemeColor null)

(** get master from mjuiThemeColor *)
let mjuiThemeColor_get_master x = Ctypes.(getf x Typs.mjuiThemeColor_master)

(** set master for mjuiThemeColor *)
let mjuiThemeColor_set_master x y = Ctypes.(setf x Typs.mjuiThemeColor_master y)

(** get thumb from mjuiThemeColor *)
let mjuiThemeColor_get_thumb x = Ctypes.(getf x Typs.mjuiThemeColor_thumb)

(** set thumb for mjuiThemeColor *)
let mjuiThemeColor_set_thumb x y = Ctypes.(setf x Typs.mjuiThemeColor_thumb y)

(** get secttitle from mjuiThemeColor *)
let mjuiThemeColor_get_secttitle x = Ctypes.(getf x Typs.mjuiThemeColor_secttitle)

(** set secttitle for mjuiThemeColor *)
let mjuiThemeColor_set_secttitle x y = Ctypes.(setf x Typs.mjuiThemeColor_secttitle y)

(** get sectfont from mjuiThemeColor *)
let mjuiThemeColor_get_sectfont x = Ctypes.(getf x Typs.mjuiThemeColor_sectfont)

(** set sectfont for mjuiThemeColor *)
let mjuiThemeColor_set_sectfont x y = Ctypes.(setf x Typs.mjuiThemeColor_sectfont y)

(** get sectsymbol from mjuiThemeColor *)
let mjuiThemeColor_get_sectsymbol x = Ctypes.(getf x Typs.mjuiThemeColor_sectsymbol)

(** set sectsymbol for mjuiThemeColor *)
let mjuiThemeColor_set_sectsymbol x y = Ctypes.(setf x Typs.mjuiThemeColor_sectsymbol y)

(** get sectpane from mjuiThemeColor *)
let mjuiThemeColor_get_sectpane x = Ctypes.(getf x Typs.mjuiThemeColor_sectpane)

(** set sectpane for mjuiThemeColor *)
let mjuiThemeColor_set_sectpane x y = Ctypes.(setf x Typs.mjuiThemeColor_sectpane y)

(** get shortcut from mjuiThemeColor *)
let mjuiThemeColor_get_shortcut x = Ctypes.(getf x Typs.mjuiThemeColor_shortcut)

(** set shortcut for mjuiThemeColor *)
let mjuiThemeColor_set_shortcut x y = Ctypes.(setf x Typs.mjuiThemeColor_shortcut y)

(** get fontactive from mjuiThemeColor *)
let mjuiThemeColor_get_fontactive x = Ctypes.(getf x Typs.mjuiThemeColor_fontactive)

(** set fontactive for mjuiThemeColor *)
let mjuiThemeColor_set_fontactive x y = Ctypes.(setf x Typs.mjuiThemeColor_fontactive y)

(** get fontinactive from mjuiThemeColor *)
let mjuiThemeColor_get_fontinactive x = Ctypes.(getf x Typs.mjuiThemeColor_fontinactive)

(** set fontinactive for mjuiThemeColor *)
let mjuiThemeColor_set_fontinactive x y =
  Ctypes.(setf x Typs.mjuiThemeColor_fontinactive y)


(** get decorinactive from mjuiThemeColor *)
let mjuiThemeColor_get_decorinactive x = Ctypes.(getf x Typs.mjuiThemeColor_decorinactive)

(** set decorinactive for mjuiThemeColor *)
let mjuiThemeColor_set_decorinactive x y =
  Ctypes.(setf x Typs.mjuiThemeColor_decorinactive y)


(** get decorinactive2 from mjuiThemeColor *)
let mjuiThemeColor_get_decorinactive2 x =
  Ctypes.(getf x Typs.mjuiThemeColor_decorinactive2)


(** set decorinactive2 for mjuiThemeColor *)
let mjuiThemeColor_set_decorinactive2 x y =
  Ctypes.(setf x Typs.mjuiThemeColor_decorinactive2 y)


(** get button from mjuiThemeColor *)
let mjuiThemeColor_get_button x = Ctypes.(getf x Typs.mjuiThemeColor_button)

(** set button for mjuiThemeColor *)
let mjuiThemeColor_set_button x y = Ctypes.(setf x Typs.mjuiThemeColor_button y)

(** get check from mjuiThemeColor *)
let mjuiThemeColor_get_check x = Ctypes.(getf x Typs.mjuiThemeColor_check)

(** set check for mjuiThemeColor *)
let mjuiThemeColor_set_check x y = Ctypes.(setf x Typs.mjuiThemeColor_check y)

(** get radio from mjuiThemeColor *)
let mjuiThemeColor_get_radio x = Ctypes.(getf x Typs.mjuiThemeColor_radio)

(** set radio for mjuiThemeColor *)
let mjuiThemeColor_set_radio x y = Ctypes.(setf x Typs.mjuiThemeColor_radio y)

(** get select from mjuiThemeColor *)
let mjuiThemeColor_get_select x = Ctypes.(getf x Typs.mjuiThemeColor_select)

(** set select for mjuiThemeColor *)
let mjuiThemeColor_set_select x y = Ctypes.(setf x Typs.mjuiThemeColor_select y)

(** get select2 from mjuiThemeColor *)
let mjuiThemeColor_get_select2 x = Ctypes.(getf x Typs.mjuiThemeColor_select2)

(** set select2 for mjuiThemeColor *)
let mjuiThemeColor_set_select2 x y = Ctypes.(setf x Typs.mjuiThemeColor_select2 y)

(** get slider from mjuiThemeColor *)
let mjuiThemeColor_get_slider x = Ctypes.(getf x Typs.mjuiThemeColor_slider)

(** set slider for mjuiThemeColor *)
let mjuiThemeColor_set_slider x y = Ctypes.(setf x Typs.mjuiThemeColor_slider y)

(** get slider2 from mjuiThemeColor *)
let mjuiThemeColor_get_slider2 x = Ctypes.(getf x Typs.mjuiThemeColor_slider2)

(** set slider2 for mjuiThemeColor *)
let mjuiThemeColor_set_slider2 x y = Ctypes.(setf x Typs.mjuiThemeColor_slider2 y)

(** get edit from mjuiThemeColor *)
let mjuiThemeColor_get_edit x = Ctypes.(getf x Typs.mjuiThemeColor_edit)

(** set edit for mjuiThemeColor *)
let mjuiThemeColor_set_edit x y = Ctypes.(setf x Typs.mjuiThemeColor_edit y)

(** get edit2 from mjuiThemeColor *)
let mjuiThemeColor_get_edit2 x = Ctypes.(getf x Typs.mjuiThemeColor_edit2)

(** set edit2 for mjuiThemeColor *)
let mjuiThemeColor_set_edit2 x y = Ctypes.(setf x Typs.mjuiThemeColor_edit2 y)

(** get cursor from mjuiThemeColor *)
let mjuiThemeColor_get_cursor x = Ctypes.(getf x Typs.mjuiThemeColor_cursor)

(** set cursor for mjuiThemeColor *)
let mjuiThemeColor_set_cursor x y = Ctypes.(setf x Typs.mjuiThemeColor_cursor y)

(** make mjuiThemeColor struct *)
let mjuiThemeColor_make
    ?mjf_master
    ?mjf_thumb
    ?mjf_secttitle
    ?mjf_sectfont
    ?mjf_sectsymbol
    ?mjf_sectpane
    ?mjf_shortcut
    ?mjf_fontactive
    ?mjf_fontinactive
    ?mjf_decorinactive
    ?mjf_decorinactive2
    ?mjf_button
    ?mjf_check
    ?mjf_radio
    ?mjf_select
    ?mjf_select2
    ?mjf_slider
    ?mjf_slider2
    ?mjf_edit
    ?mjf_edit2
    ?mjf_cursor
    ()
  =
  let x = mjuiThemeColor_allocate () in
  Option.iter (mjuiThemeColor_set_master x) mjf_master;
  Option.iter (mjuiThemeColor_set_thumb x) mjf_thumb;
  Option.iter (mjuiThemeColor_set_secttitle x) mjf_secttitle;
  Option.iter (mjuiThemeColor_set_sectfont x) mjf_sectfont;
  Option.iter (mjuiThemeColor_set_sectsymbol x) mjf_sectsymbol;
  Option.iter (mjuiThemeColor_set_sectpane x) mjf_sectpane;
  Option.iter (mjuiThemeColor_set_shortcut x) mjf_shortcut;
  Option.iter (mjuiThemeColor_set_fontactive x) mjf_fontactive;
  Option.iter (mjuiThemeColor_set_fontinactive x) mjf_fontinactive;
  Option.iter (mjuiThemeColor_set_decorinactive x) mjf_decorinactive;
  Option.iter (mjuiThemeColor_set_decorinactive2 x) mjf_decorinactive2;
  Option.iter (mjuiThemeColor_set_button x) mjf_button;
  Option.iter (mjuiThemeColor_set_check x) mjf_check;
  Option.iter (mjuiThemeColor_set_radio x) mjf_radio;
  Option.iter (mjuiThemeColor_set_select x) mjf_select;
  Option.iter (mjuiThemeColor_set_select2 x) mjf_select2;
  Option.iter (mjuiThemeColor_set_slider x) mjf_slider;
  Option.iter (mjuiThemeColor_set_slider2 x) mjf_slider2;
  Option.iter (mjuiThemeColor_set_edit x) mjf_edit;
  Option.iter (mjuiThemeColor_set_edit2 x) mjf_edit2;
  Option.iter (mjuiThemeColor_set_cursor x) mjf_cursor;
  x


(** returns mjuiThemeColor carray from list of mjuiThemeColor *)
let mjuiThemeColor_to_carray = Ctypes.CArray.of_list Typs.mjuiThemeColor

(** allocate fresh mjuiItemSingle struct *)
let mjuiItemSingle_allocate () = Ctypes.(make Typs.mjuiItemSingle)

(** make null mjuiItemSingle struct ptr *)
let mjuiItemSingle_null () = Ctypes.(from_voidp Typs.mjuiItemSingle null)

(** get modifier from mjuiItemSingle *)
let mjuiItemSingle_get_modifier x = Ctypes.(getf x Typs.mjuiItemSingle_modifier)

(** set modifier for mjuiItemSingle *)
let mjuiItemSingle_set_modifier x y = Ctypes.(setf x Typs.mjuiItemSingle_modifier y)

(** get shortcut from mjuiItemSingle *)
let mjuiItemSingle_get_shortcut x = Ctypes.(getf x Typs.mjuiItemSingle_shortcut)

(** set shortcut for mjuiItemSingle *)
let mjuiItemSingle_set_shortcut x y = Ctypes.(setf x Typs.mjuiItemSingle_shortcut y)

(** make mjuiItemSingle struct *)
let mjuiItemSingle_make ?mjf_modifier ?mjf_shortcut () =
  let x = mjuiItemSingle_allocate () in
  Option.iter (mjuiItemSingle_set_modifier x) mjf_modifier;
  Option.iter (mjuiItemSingle_set_shortcut x) mjf_shortcut;
  x


(** returns mjuiItemSingle carray from list of mjuiItemSingle *)
let mjuiItemSingle_to_carray = Ctypes.CArray.of_list Typs.mjuiItemSingle

(** allocate fresh mjuiItemMulti struct *)
let mjuiItemMulti_allocate () = Ctypes.(make Typs.mjuiItemMulti)

(** make null mjuiItemMulti struct ptr *)
let mjuiItemMulti_null () = Ctypes.(from_voidp Typs.mjuiItemMulti null)

(** get nelem from mjuiItemMulti *)
let mjuiItemMulti_get_nelem x = Ctypes.(getf x Typs.mjuiItemMulti_nelem)

(** set nelem for mjuiItemMulti *)
let mjuiItemMulti_set_nelem x y = Ctypes.(setf x Typs.mjuiItemMulti_nelem y)

(** make mjuiItemMulti struct *)
let mjuiItemMulti_make ?mjf_nelem () =
  let x = mjuiItemMulti_allocate () in
  Option.iter (mjuiItemMulti_set_nelem x) mjf_nelem;
  x


(** returns mjuiItemMulti carray from list of mjuiItemMulti *)
let mjuiItemMulti_to_carray = Ctypes.CArray.of_list Typs.mjuiItemMulti

(** allocate fresh mjuiItemSlider struct *)
let mjuiItemSlider_allocate () = Ctypes.(make Typs.mjuiItemSlider)

(** make null mjuiItemSlider struct ptr *)
let mjuiItemSlider_null () = Ctypes.(from_voidp Typs.mjuiItemSlider null)

(** get range from mjuiItemSlider *)
let mjuiItemSlider_get_range x = Ctypes.(getf x Typs.mjuiItemSlider_range)

(** set range for mjuiItemSlider *)
let mjuiItemSlider_set_range x y = Ctypes.(setf x Typs.mjuiItemSlider_range y)

(** get divisions from mjuiItemSlider *)
let mjuiItemSlider_get_divisions x = Ctypes.(getf x Typs.mjuiItemSlider_divisions)

(** set divisions for mjuiItemSlider *)
let mjuiItemSlider_set_divisions x y = Ctypes.(setf x Typs.mjuiItemSlider_divisions y)

(** make mjuiItemSlider struct *)
let mjuiItemSlider_make ?mjf_range ?mjf_divisions () =
  let x = mjuiItemSlider_allocate () in
  Option.iter (mjuiItemSlider_set_range x) mjf_range;
  Option.iter (mjuiItemSlider_set_divisions x) mjf_divisions;
  x


(** returns mjuiItemSlider carray from list of mjuiItemSlider *)
let mjuiItemSlider_to_carray = Ctypes.CArray.of_list Typs.mjuiItemSlider

(** allocate fresh mjuiItemEdit struct *)
let mjuiItemEdit_allocate () = Ctypes.(make Typs.mjuiItemEdit)

(** make null mjuiItemEdit struct ptr *)
let mjuiItemEdit_null () = Ctypes.(from_voidp Typs.mjuiItemEdit null)

(** get nelem from mjuiItemEdit *)
let mjuiItemEdit_get_nelem x = Ctypes.(getf x Typs.mjuiItemEdit_nelem)

(** set nelem for mjuiItemEdit *)
let mjuiItemEdit_set_nelem x y = Ctypes.(setf x Typs.mjuiItemEdit_nelem y)

(** make mjuiItemEdit struct *)
let mjuiItemEdit_make ?mjf_nelem () =
  let x = mjuiItemEdit_allocate () in
  Option.iter (mjuiItemEdit_set_nelem x) mjf_nelem;
  x


(** returns mjuiItemEdit carray from list of mjuiItemEdit *)
let mjuiItemEdit_to_carray = Ctypes.CArray.of_list Typs.mjuiItemEdit

(** allocate fresh mjuiSection struct *)
let mjuiSection_allocate () = Ctypes.(make Typs.mjuiSection)

(** make null mjuiSection struct ptr *)
let mjuiSection_null () = Ctypes.(from_voidp Typs.mjuiSection null)

(** get name from mjuiSection *)
let mjuiSection_get_name x = Ctypes.(getf x Typs.mjuiSection_name)

(** set name for mjuiSection *)
let mjuiSection_set_name x y = Ctypes.(setf x Typs.mjuiSection_name y)

(** get state from mjuiSection *)
let mjuiSection_get_state x = Ctypes.(getf x Typs.mjuiSection_state)

(** set state for mjuiSection *)
let mjuiSection_set_state x y = Ctypes.(setf x Typs.mjuiSection_state y)

(** get modifier from mjuiSection *)
let mjuiSection_get_modifier x = Ctypes.(getf x Typs.mjuiSection_modifier)

(** set modifier for mjuiSection *)
let mjuiSection_set_modifier x y = Ctypes.(setf x Typs.mjuiSection_modifier y)

(** get shortcut from mjuiSection *)
let mjuiSection_get_shortcut x = Ctypes.(getf x Typs.mjuiSection_shortcut)

(** set shortcut for mjuiSection *)
let mjuiSection_set_shortcut x y = Ctypes.(setf x Typs.mjuiSection_shortcut y)

(** get nitem from mjuiSection *)
let mjuiSection_get_nitem x = Ctypes.(getf x Typs.mjuiSection_nitem)

(** set nitem for mjuiSection *)
let mjuiSection_set_nitem x y = Ctypes.(setf x Typs.mjuiSection_nitem y)

(** get item from mjuiSection *)
let mjuiSection_get_item x = Ctypes.(getf x Typs.mjuiSection_item)

(** set item for mjuiSection *)
let mjuiSection_set_item x y = Ctypes.(setf x Typs.mjuiSection_item y)

(** get rtitle from mjuiSection *)
let mjuiSection_get_rtitle x = Ctypes.(getf x Typs.mjuiSection_rtitle)

(** set rtitle for mjuiSection *)
let mjuiSection_set_rtitle x y = Ctypes.(setf x Typs.mjuiSection_rtitle y)

(** get rcontent from mjuiSection *)
let mjuiSection_get_rcontent x = Ctypes.(getf x Typs.mjuiSection_rcontent)

(** set rcontent for mjuiSection *)
let mjuiSection_set_rcontent x y = Ctypes.(setf x Typs.mjuiSection_rcontent y)

(** make mjuiSection struct *)
let mjuiSection_make
    ?mjf_name
    ?mjf_state
    ?mjf_modifier
    ?mjf_shortcut
    ?mjf_nitem
    ?mjf_item
    ?mjf_rtitle
    ?mjf_rcontent
    ()
  =
  let x = mjuiSection_allocate () in
  Option.iter (mjuiSection_set_name x) mjf_name;
  Option.iter (mjuiSection_set_state x) mjf_state;
  Option.iter (mjuiSection_set_modifier x) mjf_modifier;
  Option.iter (mjuiSection_set_shortcut x) mjf_shortcut;
  Option.iter (mjuiSection_set_nitem x) mjf_nitem;
  Option.iter (mjuiSection_set_item x) mjf_item;
  Option.iter (mjuiSection_set_rtitle x) mjf_rtitle;
  Option.iter (mjuiSection_set_rcontent x) mjf_rcontent;
  x


(** returns mjuiSection carray from list of mjuiSection *)
let mjuiSection_to_carray = Ctypes.CArray.of_list Typs.mjuiSection

(** allocate fresh mjUI struct *)
let mjUI_allocate () = Ctypes.(make Typs.mjUI)

(** make null mjUI struct ptr *)
let mjUI_null () = Ctypes.(from_voidp Typs.mjUI null)

(** get spacing from mjUI *)
let mjUI_get_spacing x = Ctypes.(getf x Typs.mjUI_spacing)

(** set spacing for mjUI *)
let mjUI_set_spacing x y = Ctypes.(setf x Typs.mjUI_spacing y)

(** get color from mjUI *)
let mjUI_get_color x = Ctypes.(getf x Typs.mjUI_color)

(** set color for mjUI *)
let mjUI_set_color x y = Ctypes.(setf x Typs.mjUI_color y)

(** get predicate from mjUI *)
let mjUI_get_predicate x = Ctypes.(getf x Typs.mjUI_predicate)

(** set predicate for mjUI *)
let mjUI_set_predicate x y = Ctypes.(setf x Typs.mjUI_predicate y)

(** get userdata from mjUI *)
let mjUI_get_userdata x = Ctypes.(getf x Typs.mjUI_userdata)

(** set userdata for mjUI *)
let mjUI_set_userdata x y = Ctypes.(setf x Typs.mjUI_userdata y)

(** get rectid from mjUI *)
let mjUI_get_rectid x = Ctypes.(getf x Typs.mjUI_rectid)

(** set rectid for mjUI *)
let mjUI_set_rectid x y = Ctypes.(setf x Typs.mjUI_rectid y)

(** get auxid from mjUI *)
let mjUI_get_auxid x = Ctypes.(getf x Typs.mjUI_auxid)

(** set auxid for mjUI *)
let mjUI_set_auxid x y = Ctypes.(setf x Typs.mjUI_auxid y)

(** get radiocol from mjUI *)
let mjUI_get_radiocol x = Ctypes.(getf x Typs.mjUI_radiocol)

(** set radiocol for mjUI *)
let mjUI_set_radiocol x y = Ctypes.(setf x Typs.mjUI_radiocol y)

(** get width from mjUI *)
let mjUI_get_width x = Ctypes.(getf x Typs.mjUI_width)

(** set width for mjUI *)
let mjUI_set_width x y = Ctypes.(setf x Typs.mjUI_width y)

(** get height from mjUI *)
let mjUI_get_height x = Ctypes.(getf x Typs.mjUI_height)

(** set height for mjUI *)
let mjUI_set_height x y = Ctypes.(setf x Typs.mjUI_height y)

(** get maxheight from mjUI *)
let mjUI_get_maxheight x = Ctypes.(getf x Typs.mjUI_maxheight)

(** set maxheight for mjUI *)
let mjUI_set_maxheight x y = Ctypes.(setf x Typs.mjUI_maxheight y)

(** get scroll from mjUI *)
let mjUI_get_scroll x = Ctypes.(getf x Typs.mjUI_scroll)

(** set scroll for mjUI *)
let mjUI_set_scroll x y = Ctypes.(setf x Typs.mjUI_scroll y)

(** get mousesect from mjUI *)
let mjUI_get_mousesect x = Ctypes.(getf x Typs.mjUI_mousesect)

(** set mousesect for mjUI *)
let mjUI_set_mousesect x y = Ctypes.(setf x Typs.mjUI_mousesect y)

(** get mouseitem from mjUI *)
let mjUI_get_mouseitem x = Ctypes.(getf x Typs.mjUI_mouseitem)

(** set mouseitem for mjUI *)
let mjUI_set_mouseitem x y = Ctypes.(setf x Typs.mjUI_mouseitem y)

(** get mousehelp from mjUI *)
let mjUI_get_mousehelp x = Ctypes.(getf x Typs.mjUI_mousehelp)

(** set mousehelp for mjUI *)
let mjUI_set_mousehelp x y = Ctypes.(setf x Typs.mjUI_mousehelp y)

(** get editsect from mjUI *)
let mjUI_get_editsect x = Ctypes.(getf x Typs.mjUI_editsect)

(** set editsect for mjUI *)
let mjUI_set_editsect x y = Ctypes.(setf x Typs.mjUI_editsect y)

(** get edititem from mjUI *)
let mjUI_get_edititem x = Ctypes.(getf x Typs.mjUI_edititem)

(** set edititem for mjUI *)
let mjUI_set_edititem x y = Ctypes.(setf x Typs.mjUI_edititem y)

(** get editcursor from mjUI *)
let mjUI_get_editcursor x = Ctypes.(getf x Typs.mjUI_editcursor)

(** set editcursor for mjUI *)
let mjUI_set_editcursor x y = Ctypes.(setf x Typs.mjUI_editcursor y)

(** get editscroll from mjUI *)
let mjUI_get_editscroll x = Ctypes.(getf x Typs.mjUI_editscroll)

(** set editscroll for mjUI *)
let mjUI_set_editscroll x y = Ctypes.(setf x Typs.mjUI_editscroll y)

(** get edittext from mjUI *)
let mjUI_get_edittext x = Ctypes.(getf x Typs.mjUI_edittext)

(** set edittext for mjUI *)
let mjUI_set_edittext x y = Ctypes.(setf x Typs.mjUI_edittext y)

(** get editchanged from mjUI *)
let mjUI_get_editchanged x = Ctypes.(getf x Typs.mjUI_editchanged)

(** set editchanged for mjUI *)
let mjUI_set_editchanged x y = Ctypes.(setf x Typs.mjUI_editchanged y)

(** get nsect from mjUI *)
let mjUI_get_nsect x = Ctypes.(getf x Typs.mjUI_nsect)

(** set nsect for mjUI *)
let mjUI_set_nsect x y = Ctypes.(setf x Typs.mjUI_nsect y)

(** get sect from mjUI *)
let mjUI_get_sect x = Ctypes.(getf x Typs.mjUI_sect)

(** set sect for mjUI *)
let mjUI_set_sect x y = Ctypes.(setf x Typs.mjUI_sect y)

(** make mjUI struct *)
let mjUI_make
    ?mjf_spacing
    ?mjf_color
    ?mjf_predicate
    ?mjf_userdata
    ?mjf_rectid
    ?mjf_auxid
    ?mjf_radiocol
    ?mjf_width
    ?mjf_height
    ?mjf_maxheight
    ?mjf_scroll
    ?mjf_mousesect
    ?mjf_mouseitem
    ?mjf_mousehelp
    ?mjf_editsect
    ?mjf_edititem
    ?mjf_editcursor
    ?mjf_editscroll
    ?mjf_edittext
    ?mjf_editchanged
    ?mjf_nsect
    ?mjf_sect
    ()
  =
  let x = mjUI_allocate () in
  Option.iter (mjUI_set_spacing x) mjf_spacing;
  Option.iter (mjUI_set_color x) mjf_color;
  Option.iter (mjUI_set_predicate x) mjf_predicate;
  Option.iter (mjUI_set_userdata x) mjf_userdata;
  Option.iter (mjUI_set_rectid x) mjf_rectid;
  Option.iter (mjUI_set_auxid x) mjf_auxid;
  Option.iter (mjUI_set_radiocol x) mjf_radiocol;
  Option.iter (mjUI_set_width x) mjf_width;
  Option.iter (mjUI_set_height x) mjf_height;
  Option.iter (mjUI_set_maxheight x) mjf_maxheight;
  Option.iter (mjUI_set_scroll x) mjf_scroll;
  Option.iter (mjUI_set_mousesect x) mjf_mousesect;
  Option.iter (mjUI_set_mouseitem x) mjf_mouseitem;
  Option.iter (mjUI_set_mousehelp x) mjf_mousehelp;
  Option.iter (mjUI_set_editsect x) mjf_editsect;
  Option.iter (mjUI_set_edititem x) mjf_edititem;
  Option.iter (mjUI_set_editcursor x) mjf_editcursor;
  Option.iter (mjUI_set_editscroll x) mjf_editscroll;
  Option.iter (mjUI_set_edittext x) mjf_edittext;
  Option.iter (mjUI_set_editchanged x) mjf_editchanged;
  Option.iter (mjUI_set_nsect x) mjf_nsect;
  Option.iter (mjUI_set_sect x) mjf_sect;
  x


(** returns mjUI carray from list of mjUI *)
let mjUI_to_carray = Ctypes.CArray.of_list Typs.mjUI

(** allocate fresh mjuiDef struct *)
let mjuiDef_allocate () = Ctypes.(make Typs.mjuiDef)

(** make null mjuiDef struct ptr *)
let mjuiDef_null () = Ctypes.(from_voidp Typs.mjuiDef null)

(** get type from mjuiDef *)
let mjuiDef_get_type x = Ctypes.(getf x Typs.mjuiDef_type)

(** set type for mjuiDef *)
let mjuiDef_set_type x y = Ctypes.(setf x Typs.mjuiDef_type y)

(** get name from mjuiDef *)
let mjuiDef_get_name x = Ctypes.(getf x Typs.mjuiDef_name)

(** set name for mjuiDef *)
let mjuiDef_set_name x y = Ctypes.(setf x Typs.mjuiDef_name y)

(** get state from mjuiDef *)
let mjuiDef_get_state x = Ctypes.(getf x Typs.mjuiDef_state)

(** set state for mjuiDef *)
let mjuiDef_set_state x y = Ctypes.(setf x Typs.mjuiDef_state y)

(** get pdata from mjuiDef *)
let mjuiDef_get_pdata x = Ctypes.(getf x Typs.mjuiDef_pdata)

(** set pdata for mjuiDef *)
let mjuiDef_set_pdata x y = Ctypes.(setf x Typs.mjuiDef_pdata y)

(** get other from mjuiDef *)
let mjuiDef_get_other x = Ctypes.(getf x Typs.mjuiDef_other)

(** set other for mjuiDef *)
let mjuiDef_set_other x y = Ctypes.(setf x Typs.mjuiDef_other y)

(** make mjuiDef struct *)
let mjuiDef_make ?mjf_type ?mjf_name ?mjf_state ?mjf_pdata ?mjf_other () =
  let x = mjuiDef_allocate () in
  Option.iter (mjuiDef_set_type x) mjf_type;
  Option.iter (mjuiDef_set_name x) mjf_name;
  Option.iter (mjuiDef_set_state x) mjf_state;
  Option.iter (mjuiDef_set_pdata x) mjf_pdata;
  Option.iter (mjuiDef_set_other x) mjf_other;
  x


(** returns mjuiDef carray from list of mjuiDef *)
let mjuiDef_to_carray = Ctypes.CArray.of_list Typs.mjuiDef

(** ---------------------- Activation ----------------------------------------------------- *)

(** Return 1 (for backward compatibility). *)
let mj_activate = Bindings.mj_activate

(** Do nothing (for backward compatibility). *)
let mj_deactivate = Bindings.mj_deactivate

(** ---------------------- Virtual file system -------------------------------------------- *)

(** Initialize VFS to empty (no deallocation). *)
let mj_defaultVFS = Bindings.mj_defaultVFS

(** Add file to VFS, return 0: success, 1: full, 2: repeated name, -1: not found on disk. *)
let mj_addFileVFS = Bindings.mj_addFileVFS

(** Make empty file in VFS, return 0: success, 1: full, 2: repeated name. *)
let mj_makeEmptyFileVFS = Bindings.mj_makeEmptyFileVFS

(** Return file index in VFS, or -1 if not found in VFS. *)
let mj_findFileVFS = Bindings.mj_findFileVFS

(** Delete file from VFS, return 0: success, -1: not found in VFS. *)
let mj_deleteFileVFS = Bindings.mj_deleteFileVFS

(** Delete all files from VFS. *)
let mj_deleteVFS = Bindings.mj_deleteVFS

(** ---------------------- Parse and compile ---------------------------------------------- *)

(** Parse XML file in MJCF or URDF format, compile it, return low-level model.
 If vfs is not NULL, look up files in vfs before reading from disk.
 If error is not NULL, it must have size error_sz. *)
let mj_loadXML = Bindings.mj_loadXML

(** Update XML data structures with info from low-level model, save as MJCF.
 If error is not NULL, it must have size error_sz. *)
let mj_saveLastXML = Bindings.mj_saveLastXML

(** Free last XML model if loaded. Called internally at each load. *)
let mj_freeLastXML = Bindings.mj_freeLastXML

(** Print internal XML schema as plain text or HTML, with style-padding or &nbsp;. *)
let mj_printSchema = Bindings.mj_printSchema

(** ---------------------- Main simulation ------------------------------------------------ *)

(** Advance simulation, use control callback to obtain external force and control. *)
let mj_step = Bindings.mj_step

(** Advance simulation in two steps: before external force and control is set by user. *)
let mj_step1 = Bindings.mj_step1

(** Advance simulation in two steps: after external force and control is set by user. *)
let mj_step2 = Bindings.mj_step2

(** Forward dynamics: same as mj_step but do not integrate in time. *)
let mj_forward = Bindings.mj_forward

(** Inverse dynamics: qacc must be set before calling. *)
let mj_inverse = Bindings.mj_inverse

(** Forward dynamics with skip; skipstage is mjtStage. *)
let mj_forwardSkip = Bindings.mj_forwardSkip

(** Inverse dynamics with skip; skipstage is mjtStage. *)
let mj_inverseSkip = Bindings.mj_inverseSkip

(** ---------------------- Initialization ------------------------------------------------- *)

(** Set default options for length range computation. *)
let mj_defaultLROpt = Bindings.mj_defaultLROpt

(** Set solver parameters to default values. *)
let mj_defaultSolRefImp = Bindings.mj_defaultSolRefImp

(** Set physics options to default values. *)
let mj_defaultOption = Bindings.mj_defaultOption

(** Set visual options to default values. *)
let mj_defaultVisual = Bindings.mj_defaultVisual

(** Copy mjModel, allocate new if dest is NULL. *)
let mj_copyModel = Bindings.mj_copyModel

(** Save model to binary MJB file or memory buffer; buffer has precedence when given. *)
let mj_saveModel = Bindings.mj_saveModel

(** Load model from binary MJB file.
 If vfs is not NULL, look up file in vfs before reading from disk. *)
let mj_loadModel = Bindings.mj_loadModel

(** Free memory allocation in model. *)
let mj_deleteModel = Bindings.mj_deleteModel

(** Return size of buffer needed to hold model. *)
let mj_sizeModel = Bindings.mj_sizeModel

(** Allocate mjData correponding to given model. *)
let mj_makeData = Bindings.mj_makeData

(** Copy mjData. *)
let mj_copyData = Bindings.mj_copyData

(** Reset data to defaults. *)
let mj_resetData = Bindings.mj_resetData

(** Reset data to defaults, fill everything else with debug_value. *)
let mj_resetDataDebug = Bindings.mj_resetDataDebug

(** Reset data, set fields from specified keyframe. *)
let mj_resetDataKeyframe = Bindings.mj_resetDataKeyframe

(** Allocate array of specified size on mjData stack. Call mju_error on stack overflow. *)
let mj_stackAlloc = Bindings.mj_stackAlloc

(** Free memory allocation in mjData. *)
let mj_deleteData = Bindings.mj_deleteData

(** Reset all callbacks to NULL pointers (NULL is the default). *)
let mj_resetCallbacks = Bindings.mj_resetCallbacks

(** Set constant fields of mjModel, corresponding to qpos0 configuration. *)
let mj_setConst = Bindings.mj_setConst

(** Set actuator_lengthrange for specified actuator; return 1 if ok, 0 if error. *)
let mj_setLengthRange = Bindings.mj_setLengthRange

(** ---------------------- Printing ------------------------------------------------------- *)

(** Print model to text file. *)
let mj_printModel = Bindings.mj_printModel

(** Print data to text file. *)
let mj_printData = Bindings.mj_printData

(** Print matrix to screen. *)
let mju_printMat = Bindings.mju_printMat

(** Print sparse matrix to screen. *)
let mju_printMatSparse = Bindings.mju_printMatSparse

(** ---------------------- Components ----------------------------------------------------- *)

(** Run position-dependent computations. *)
let mj_fwdPosition = Bindings.mj_fwdPosition

(** Run velocity-dependent computations. *)
let mj_fwdVelocity = Bindings.mj_fwdVelocity

(** Compute actuator force qfrc_actuation. *)
let mj_fwdActuation = Bindings.mj_fwdActuation

(** Add up all non-constraint forces, compute qacc_unc. *)
let mj_fwdAcceleration = Bindings.mj_fwdAcceleration

(** Run selected constraint solver. *)
let mj_fwdConstraint = Bindings.mj_fwdConstraint

(** Euler integrator, semi-implicit in velocity. *)
let mj_Euler = Bindings.mj_Euler

(** Runge-Kutta explicit order-N integrator. *)
let mj_RungeKutta = Bindings.mj_RungeKutta

(** Run position-dependent computations in inverse dynamics. *)
let mj_invPosition = Bindings.mj_invPosition

(** Run velocity-dependent computations in inverse dynamics. *)
let mj_invVelocity = Bindings.mj_invVelocity

(** Apply the analytical formula for inverse constraint dynamics. *)
let mj_invConstraint = Bindings.mj_invConstraint

(** Compare forward and inverse dynamics, save results in fwdinv. *)
let mj_compareFwdInv = Bindings.mj_compareFwdInv

(** ---------------------- Sub components ------------------------------------------------- *)

(** Evaluate position-dependent sensors. *)
let mj_sensorPos = Bindings.mj_sensorPos

(** Evaluate velocity-dependent sensors. *)
let mj_sensorVel = Bindings.mj_sensorVel

(** Evaluate acceleration and force-dependent sensors. *)
let mj_sensorAcc = Bindings.mj_sensorAcc

(** Evaluate position-dependent energy (potential). *)
let mj_energyPos = Bindings.mj_energyPos

(** Evaluate velocity-dependent energy (kinetic). *)
let mj_energyVel = Bindings.mj_energyVel

(** Check qpos, reset if any element is too big or nan. *)
let mj_checkPos = Bindings.mj_checkPos

(** Check qvel, reset if any element is too big or nan. *)
let mj_checkVel = Bindings.mj_checkVel

(** Check qacc, reset if any element is too big or nan. *)
let mj_checkAcc = Bindings.mj_checkAcc

(** Run forward kinematics. *)
let mj_kinematics = Bindings.mj_kinematics

(** Map inertias and motion dofs to global frame centered at CoM. *)
let mj_comPos = Bindings.mj_comPos

(** Compute camera and light positions and orientations. *)
let mj_camlight = Bindings.mj_camlight

(** Compute tendon lengths, velocities and moment arms. *)
let mj_tendon = Bindings.mj_tendon

(** Compute actuator transmission lengths and moments. *)
let mj_transmission = Bindings.mj_transmission

(** Run composite rigid body inertia algorithm (CRB). *)
let mj_crb = Bindings.mj_crb

(** Compute sparse L'*D*L factorizaton of inertia matrix. *)
let mj_factorM = Bindings.mj_factorM

(** Solve linear system M * x = y using factorization:  x = inv(L'*D*L)*y *)
let mj_solveM = Bindings.mj_solveM

(** Half of linear solve:  x = sqrt(inv(D))*inv(L')*y *)
let mj_solveM2 = Bindings.mj_solveM2

(** Compute cvel, cdof_dot. *)
let mj_comVel = Bindings.mj_comVel

(** Compute qfrc_passive from spring-dampers, viscosity and density. *)
let mj_passive = Bindings.mj_passive

(** subtree linear velocity and angular momentum *)
let mj_subtreeVel = Bindings.mj_subtreeVel

(** RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term. *)
let mj_rne = Bindings.mj_rne

(** RNE with complete data: compute cacc, cfrc_ext, cfrc_int. *)
let mj_rnePostConstraint = Bindings.mj_rnePostConstraint

(** Run collision detection. *)
let mj_collision = Bindings.mj_collision

(** Construct constraints. *)
let mj_makeConstraint = Bindings.mj_makeConstraint

(** Compute inverse constaint inertia efc_AR. *)
let mj_projectConstraint = Bindings.mj_projectConstraint

(** Compute efc_vel, efc_aref. *)
let mj_referenceConstraint = Bindings.mj_referenceConstraint

(** Compute efc_state, efc_force, qfrc_constraint, and (optionally) cone Hessians.
 If cost is not NULL, set *cost = s(jar) where jar = Jac*qacc-aref. *)
let mj_constraintUpdate = Bindings.mj_constraintUpdate

(** ---------------------- Support -------------------------------------------------------- *)

(** Add contact to d->contact list; return 0 if success; 1 if buffer full. *)
let mj_addContact = Bindings.mj_addContact

(** Determine type of friction cone. *)
let mj_isPyramidal = Bindings.mj_isPyramidal

(** Determine type of constraint Jacobian. *)
let mj_isSparse = Bindings.mj_isSparse

(** Determine type of solver (PGS is dual, CG and Newton are primal). *)
let mj_isDual = Bindings.mj_isDual

(** Multiply dense or sparse constraint Jacobian by vector. *)
let mj_mulJacVec = Bindings.mj_mulJacVec

(** Multiply dense or sparse constraint Jacobian transpose by vector. *)
let mj_mulJacTVec = Bindings.mj_mulJacTVec

(** Compute 3/6-by-nv end-effector Jacobian of global point attached to given body. *)
let mj_jac = Bindings.mj_jac

(** Compute body frame end-effector Jacobian. *)
let mj_jacBody = Bindings.mj_jacBody

(** Compute body center-of-mass end-effector Jacobian. *)
let mj_jacBodyCom = Bindings.mj_jacBodyCom

(** Compute geom end-effector Jacobian. *)
let mj_jacGeom = Bindings.mj_jacGeom

(** Compute site end-effector Jacobian. *)
let mj_jacSite = Bindings.mj_jacSite

(** Compute translation end-effector Jacobian of point, and rotation Jacobian of axis. *)
let mj_jacPointAxis = Bindings.mj_jacPointAxis

(** Get id of object with specified name, return -1 if not found; type is mjtObj. *)
let mj_name2id = Bindings.mj_name2id

(** Convert sparse inertia matrix M into full (i.e. dense) matrix. *)
let mj_fullM = Bindings.mj_fullM

(** Multiply vector by inertia matrix. *)
let mj_mulM = Bindings.mj_mulM

(** Multiply vector by (inertia matrix)^(1/2). *)
let mj_mulM2 = Bindings.mj_mulM2

(** Add inertia matrix to destination matrix.
 Destination can be sparse uncompressed, or dense when all int* are NULL *)
let mj_addM = Bindings.mj_addM

(** Apply cartesian force and torque (outside xfrc_applied mechanism). *)
let mj_applyFT = Bindings.mj_applyFT

(** Compute object 6D velocity in object-centered frame, world/local orientation. *)
let mj_objectVelocity = Bindings.mj_objectVelocity

(** Compute object 6D acceleration in object-centered frame, world/local orientation. *)
let mj_objectAcceleration = Bindings.mj_objectAcceleration

(** Extract 6D force:torque for one contact, in contact frame. *)
let mj_contactForce = Bindings.mj_contactForce

(** Compute velocity by finite-differencing two positions. *)
let mj_differentiatePos = Bindings.mj_differentiatePos

(** Integrate position with given velocity. *)
let mj_integratePos = Bindings.mj_integratePos

(** Normalize all quaterions in qpos-type vector. *)
let mj_normalizeQuat = Bindings.mj_normalizeQuat

(** Map from body local to global Cartesian coordinates. *)
let mj_local2Global = Bindings.mj_local2Global

(** Sum all body masses. *)
let mj_getTotalmass = Bindings.mj_getTotalmass

(** Scale body masses and inertias to achieve specified total mass. *)
let mj_setTotalmass = Bindings.mj_setTotalmass

(** Return version number: 1.0.2 is encoded as 102. *)
let mj_version = Bindings.mj_version

(** ---------------------- Ray collisions ------------------------------------------------- *)

(** Intersect ray (pnt+x*vec, x>=0) with visible geoms, except geoms in bodyexclude.
 Return geomid and distance (x) to nearest surface, or -1 if no intersection.
 geomgroup, flg_static are as in mjvOption; geomgroup==NULL skips group exclusion. *)
let mj_ray = Bindings.mj_ray

(** Interect ray with hfield, return nearest distance or -1 if no intersection. *)
let mj_rayHfield = Bindings.mj_rayHfield

(** Interect ray with mesh, return nearest distance or -1 if no intersection. *)
let mj_rayMesh = Bindings.mj_rayMesh

(** Interect ray with pure geom, return nearest distance or -1 if no intersection. *)
let mju_rayGeom = Bindings.mju_rayGeom

(** Interect ray with skin, return nearest vertex id. *)
let mju_raySkin = Bindings.mju_raySkin

(** ---------------------- Interaction ---------------------------------------------------- *)

(** Set default camera. *)
let mjv_defaultCamera = Bindings.mjv_defaultCamera

(** Set default perturbation. *)
let mjv_defaultPerturb = Bindings.mjv_defaultPerturb

(** Transform pose from room to model space. *)
let mjv_room2model = Bindings.mjv_room2model

(** Transform pose from model to room space. *)
let mjv_model2room = Bindings.mjv_model2room

(** Get camera info in model space; average left and right OpenGL cameras. *)
let mjv_cameraInModel = Bindings.mjv_cameraInModel

(** Get camera info in room space; average left and right OpenGL cameras. *)
let mjv_cameraInRoom = Bindings.mjv_cameraInRoom

(** Get frustum height at unit distance from camera; average left and right OpenGL cameras. *)
let mjv_frustumHeight = Bindings.mjv_frustumHeight

(** Rotate 3D vec in horizontal plane by angle between (0,1) and (forward_x,forward_y). *)
let mjv_alignToCamera = Bindings.mjv_alignToCamera

(** Move camera with mouse; action is mjtMouse. *)
let mjv_moveCamera = Bindings.mjv_moveCamera

(** Move perturb object with mouse; action is mjtMouse. *)
let mjv_movePerturb = Bindings.mjv_movePerturb

(** Move model with mouse; action is mjtMouse. *)
let mjv_moveModel = Bindings.mjv_moveModel

(** Copy perturb pos,quat from selected body; set scale for perturbation. *)
let mjv_initPerturb = Bindings.mjv_initPerturb

(** Set perturb pos,quat in d->mocap when selected body is mocap, and in d->qpos otherwise.
 Write d->qpos only if flg_paused and subtree root for selected body has free joint. *)
let mjv_applyPerturbPose = Bindings.mjv_applyPerturbPose

(** Set perturb force,torque in d->xfrc_applied, if selected body is dynamic. *)
let mjv_applyPerturbForce = Bindings.mjv_applyPerturbForce

(** Return the average of two OpenGL cameras. *)
let mjv_averageCamera = Bindings.mjv_averageCamera

(** Select geom or skin with mouse, return bodyid; -1: none selected. *)
let mjv_select = Bindings.mjv_select

(** ---------------------- Visualization -------------------------------------------------- *)

(** Set default visualization options. *)
let mjv_defaultOption = Bindings.mjv_defaultOption

(** Set default figure. *)
let mjv_defaultFigure = Bindings.mjv_defaultFigure

(** Initialize given geom fields when not NULL, set the rest to their default values. *)
let mjv_initGeom = Bindings.mjv_initGeom

(** Set (type, size, pos, mat) for connector-type geom between given points.
 Assume that mjv_initGeom was already called to set all other properties. *)
let mjv_makeConnector = Bindings.mjv_makeConnector

(** Set default abstract scene. *)
let mjv_defaultScene = Bindings.mjv_defaultScene

(** Allocate resources in abstract scene. *)
let mjv_makeScene = Bindings.mjv_makeScene

(** Free abstract scene. *)
let mjv_freeScene = Bindings.mjv_freeScene

(** Update entire scene given model state. *)
let mjv_updateScene = Bindings.mjv_updateScene

(** Add geoms from selected categories. *)
let mjv_addGeoms = Bindings.mjv_addGeoms

(** Make list of lights. *)
let mjv_makeLights = Bindings.mjv_makeLights

(** Update camera. *)
let mjv_updateCamera = Bindings.mjv_updateCamera

(** Update skins. *)
let mjv_updateSkin = Bindings.mjv_updateSkin

(** ---------------------- OpenGL rendering ----------------------------------------------- *)

(** Set default mjrContext. *)
let mjr_defaultContext = Bindings.mjr_defaultContext

(** Allocate resources in custom OpenGL context; fontscale is mjtFontScale. *)
let mjr_makeContext = Bindings.mjr_makeContext

(** Change font of existing context. *)
let mjr_changeFont = Bindings.mjr_changeFont

(** Add Aux buffer with given index to context; free previous Aux buffer. *)
let mjr_addAux = Bindings.mjr_addAux

(** Free resources in custom OpenGL context, set to default. *)
let mjr_freeContext = Bindings.mjr_freeContext

(** Upload texture to GPU, overwriting previous upload if any. *)
let mjr_uploadTexture = Bindings.mjr_uploadTexture

(** Upload mesh to GPU, overwriting previous upload if any. *)
let mjr_uploadMesh = Bindings.mjr_uploadMesh

(** Upload height field to GPU, overwriting previous upload if any. *)
let mjr_uploadHField = Bindings.mjr_uploadHField

(** Make con->currentBuffer current again. *)
let mjr_restoreBuffer = Bindings.mjr_restoreBuffer

(** Set OpenGL framebuffer for rendering: mjFB_WINDOW or mjFB_OFFSCREEN.
 If only one buffer is available, set that buffer and ignore framebuffer argument. *)
let mjr_setBuffer = Bindings.mjr_setBuffer

(** Read pixels from current OpenGL framebuffer to client buffer.
 Viewport is in OpenGL framebuffer; client buffer starts at (0,0). *)
let mjr_readPixels = Bindings.mjr_readPixels

(** Draw pixels from client buffer to current OpenGL framebuffer.
 Viewport is in OpenGL framebuffer; client buffer starts at (0,0). *)
let mjr_drawPixels = Bindings.mjr_drawPixels

(** Blit from src viewpoint in current framebuffer to dst viewport in other framebuffer.
 If src, dst have different size and flg_depth==0, color is interpolated with GL_LINEAR. *)
let mjr_blitBuffer = Bindings.mjr_blitBuffer

(** Set Aux buffer for custom OpenGL rendering (call restoreBuffer when done). *)
let mjr_setAux = Bindings.mjr_setAux

(** Blit from Aux buffer to con->currentBuffer. *)
let mjr_blitAux = Bindings.mjr_blitAux

(** Draw text at (x,y) in relative coordinates; font is mjtFont. *)
let mjr_text = Bindings.mjr_text

(** Draw text overlay; font is mjtFont; gridpos is mjtGridPos. *)
let mjr_overlay = Bindings.mjr_overlay

(** Get maximum viewport for active buffer. *)
let mjr_maxViewport = Bindings.mjr_maxViewport

(** Draw rectangle. *)
let mjr_rectangle = Bindings.mjr_rectangle

(** Draw rectangle with centered text. *)
let mjr_label = Bindings.mjr_label

(** Draw 2D figure. *)
let mjr_figure = Bindings.mjr_figure

(** Render 3D scene. *)
let mjr_render = Bindings.mjr_render

(** Call glFinish. *)
let mjr_finish = Bindings.mjr_finish

(** Call glGetError and return result. *)
let mjr_getError = Bindings.mjr_getError

(** Find first rectangle containing mouse, -1: not found. *)
let mjr_findRect = Bindings.mjr_findRect

(** ---------------------- UI framework --------------------------------------------------- *)

(** Get builtin UI theme spacing (ind: 0-1). *)
let mjui_themeSpacing = Bindings.mjui_themeSpacing

(** Get builtin UI theme color (ind: 0-3). *)
let mjui_themeColor = Bindings.mjui_themeColor

(** Add definitions to UI. *)
let mjui_add = Bindings.mjui_add

(** Add definitions to UI section. *)
let mjui_addToSection = Bindings.mjui_addToSection

(** Compute UI sizes. *)
let mjui_resize = Bindings.mjui_resize

(** Update specific section/item; -1: update all. *)
let mjui_update = Bindings.mjui_update

(** Handle UI event, return pointer to changed item, NULL if no change. *)
let mjui_event = Bindings.mjui_event

(** Copy UI image to current buffer. *)
let mjui_render = Bindings.mjui_render

(** ---------------------- Error and memory ----------------------------------------------- *)

(** Main error function; does not return to caller. *)
let mju_error = Bindings.mju_error

(** Error function with int argument; msg is a printf format string. *)
let mju_error_i = Bindings.mju_error_i

(** Error function with string argument. *)
let mju_error_s = Bindings.mju_error_s

(** Main warning function; returns to caller. *)
let mju_warning = Bindings.mju_warning

(** Warning function with int argument. *)
let mju_warning_i = Bindings.mju_warning_i

(** Warning function with string argument. *)
let mju_warning_s = Bindings.mju_warning_s

(** Clear user error and memory handlers. *)
let mju_clearHandlers = Bindings.mju_clearHandlers

(** Allocate memory; byte-align on 8; pad size to multiple of 8. *)
let mju_malloc = Bindings.mju_malloc

(** Free memory, using free() by default. *)
let mju_free = Bindings.mju_free

(** High-level warning function: count warnings in mjData, print only the first. *)
let mj_warning = Bindings.mj_warning

(** Write [datetime, type: message] to MUJOCO_LOG.TXT. *)
let mju_writeLog = Bindings.mju_writeLog

(** ---------------------- Standard math -------------------------------------------------- *)

(** ------------------------------ Vector math -------------------------------------------- *)

(** Set res = 0. *)
let mju_zero3 = Bindings.mju_zero3

(** Set res = vec. *)
let mju_copy3 = Bindings.mju_copy3

(** Set res = vec*scl. *)
let mju_scl3 = Bindings.mju_scl3

(** Set res = vec1 + vec2. *)
let mju_add3 = Bindings.mju_add3

(** Set res = vec1 - vec2. *)
let mju_sub3 = Bindings.mju_sub3

(** Set res = res + vec. *)
let mju_addTo3 = Bindings.mju_addTo3

(** Set res = res - vec. *)
let mju_subFrom3 = Bindings.mju_subFrom3

(** Set res = res + vec*scl. *)
let mju_addToScl3 = Bindings.mju_addToScl3

(** Set res = vec1 + vec2*scl. *)
let mju_addScl3 = Bindings.mju_addScl3

(** Normalize vector, return length before normalization. *)
let mju_normalize3 = Bindings.mju_normalize3

(** Return vector length (without normalizing the vector). *)
let mju_norm3 = Bindings.mju_norm3

(** Return dot-product of vec1 and vec2. *)
let mju_dot3 = Bindings.mju_dot3

(** Return Cartesian distance between 3D vectors pos1 and pos2. *)
let mju_dist3 = Bindings.mju_dist3

(** Multiply vector by 3D rotation matrix: res = mat * vec. *)
let mju_rotVecMat = Bindings.mju_rotVecMat

(** Multiply vector by transposed 3D rotation matrix: res = mat' * vec. *)
let mju_rotVecMatT = Bindings.mju_rotVecMatT

(** Compute cross-product: res = cross(a, b). *)
let mju_cross = Bindings.mju_cross

(** Set res = 0. *)
let mju_zero4 = Bindings.mju_zero4

(** Set res = (1,0,0,0). *)
let mju_unit4 = Bindings.mju_unit4

(** Set res = vec. *)
let mju_copy4 = Bindings.mju_copy4

(** Normalize vector, return length before normalization. *)
let mju_normalize4 = Bindings.mju_normalize4

(** Set res = 0. *)
let mju_zero = Bindings.mju_zero

(** Set res = vec. *)
let mju_copy = Bindings.mju_copy

(** Return sum(vec). *)
let mju_sum = Bindings.mju_sum

(** Return L1 norm: sum(abs(vec)). *)
let mju_L1 = Bindings.mju_L1

(** Set res = vec*scl. *)
let mju_scl = Bindings.mju_scl

(** Set res = vec1 + vec2. *)
let mju_add = Bindings.mju_add

(** Set res = vec1 - vec2. *)
let mju_sub = Bindings.mju_sub

(** Set res = res + vec. *)
let mju_addTo = Bindings.mju_addTo

(** Set res = res - vec. *)
let mju_subFrom = Bindings.mju_subFrom

(** Set res = res + vec*scl. *)
let mju_addToScl = Bindings.mju_addToScl

(** Set res = vec1 + vec2*scl. *)
let mju_addScl = Bindings.mju_addScl

(** Normalize vector, return length before normalization. *)
let mju_normalize = Bindings.mju_normalize

(** Return vector length (without normalizing vector). *)
let mju_norm = Bindings.mju_norm

(** Return dot-product of vec1 and vec2. *)
let mju_dot = Bindings.mju_dot

(** Multiply matrix and vector: res = mat * vec. *)
let mju_mulMatVec = Bindings.mju_mulMatVec

(** Multiply transposed matrix and vector: res = mat' * vec. *)
let mju_mulMatTVec = Bindings.mju_mulMatTVec

(** Transpose matrix: res = mat'. *)
let mju_transpose = Bindings.mju_transpose

(** Multiply matrices: res = mat1 * mat2. *)
let mju_mulMatMat = Bindings.mju_mulMatMat

(** Multiply matrices, second argument transposed: res = mat1 * mat2'. *)
let mju_mulMatMatT = Bindings.mju_mulMatMatT

(** Multiply matrices, first argument transposed: res = mat1' * mat2. *)
let mju_mulMatTMat = Bindings.mju_mulMatTMat

(** Set res = mat' * diag * mat if diag is not NULL, and res = mat' * mat otherwise. *)
let mju_sqrMatTD = Bindings.mju_sqrMatTD

(** Coordinate transform of 6D motion or force vector in rotation:translation format.
 rotnew2old is 3-by-3, NULL means no rotation; flg_force specifies force or motion type. *)
let mju_transformSpatial = Bindings.mju_transformSpatial

(** ---------------------- Quaternions ---------------------------------------------------- *)

(** Rotate vector by quaternion. *)
let mju_rotVecQuat = Bindings.mju_rotVecQuat

(** Conjugate quaternion, corresponding to opposite rotation. *)
let mju_negQuat = Bindings.mju_negQuat

(** Muiltiply quaternions. *)
let mju_mulQuat = Bindings.mju_mulQuat

(** Muiltiply quaternion and axis. *)
let mju_mulQuatAxis = Bindings.mju_mulQuatAxis

(** Convert axisAngle to quaternion. *)
let mju_axisAngle2Quat = Bindings.mju_axisAngle2Quat

(** Convert quaternion (corresponding to orientation difference) to 3D velocity. *)
let mju_quat2Vel = Bindings.mju_quat2Vel

(** Subtract quaternions, express as 3D velocity: qb*quat(res) = qa. *)
let mju_subQuat = Bindings.mju_subQuat

(** Convert quaternion to 3D rotation matrix. *)
let mju_quat2Mat = Bindings.mju_quat2Mat

(** Convert 3D rotation matrix to quaterion. *)
let mju_mat2Quat = Bindings.mju_mat2Quat

(** Compute time-derivative of quaternion, given 3D rotational velocity. *)
let mju_derivQuat = Bindings.mju_derivQuat

(** Integrate quaterion given 3D angular velocity. *)
let mju_quatIntegrate = Bindings.mju_quatIntegrate

(** Construct quaternion performing rotation from z-axis to given vector. *)
let mju_quatZ2Vec = Bindings.mju_quatZ2Vec

(** ---------------------- Poses ---------------------------------------------------------- *)

(** Multiply two poses. *)
let mju_mulPose = Bindings.mju_mulPose

(** Conjugate pose, corresponding to the opposite spatial transformation. *)
let mju_negPose = Bindings.mju_negPose

(** Transform vector by pose. *)
let mju_trnVecPose = Bindings.mju_trnVecPose

(** ---------------------- Decompositions -------------------------------------------------- *)

(** Cholesky decomposition: mat = L*L'; return rank. *)
let mju_cholFactor = Bindings.mju_cholFactor

(** Solve mat * res = vec, where mat is Cholesky-factorized *)
let mju_cholSolve = Bindings.mju_cholSolve

(** Cholesky rank-one update: L*L' +/- x*x'; return rank. *)
let mju_cholUpdate = Bindings.mju_cholUpdate

(** Eigenvalue decomposition of symmetric 3x3 matrix. *)
let mju_eig3 = Bindings.mju_eig3

(** ---------------------- Miscellaneous -------------------------------------------------- *)

(** Muscle active force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax). *)
let mju_muscleGain = Bindings.mju_muscleGain

(** Muscle passive force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax). *)
let mju_muscleBias = Bindings.mju_muscleBias

(** Muscle activation dynamics, prm = (tau_act, tau_deact). *)
let mju_muscleDynamics = Bindings.mju_muscleDynamics

(** Convert contact force to pyramid representation. *)
let mju_encodePyramid = Bindings.mju_encodePyramid

(** Convert pyramid representation to contact force. *)
let mju_decodePyramid = Bindings.mju_decodePyramid

(** Integrate spring-damper analytically, return pos(dt). *)
let mju_springDamper = Bindings.mju_springDamper

(** Return min(a,b) with single evaluation of a and b. *)
let mju_min = Bindings.mju_min

(** Return max(a,b) with single evaluation of a and b. *)
let mju_max = Bindings.mju_max

(** Return sign of x: +1, -1 or 0. *)
let mju_sign = Bindings.mju_sign

(** Round x to nearest integer. *)
let mju_round = Bindings.mju_round

(** Convert type name to type id (mjtObj). *)
let mju_str2Type = Bindings.mju_str2Type

(** Return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise. Used by check functions. *)
let mju_isBad = Bindings.mju_isBad

(** Return 1 if all elements are 0. *)
let mju_isZero = Bindings.mju_isZero

(** Standard normal random number generator (optional second number). *)
let mju_standardNormal = Bindings.mju_standardNormal

(** Convert from float to mjtNum. *)
let mju_f2n = Bindings.mju_f2n

(** Convert from mjtNum to float. *)
let mju_n2f = Bindings.mju_n2f

(** Convert from double to mjtNum. *)
let mju_d2n = Bindings.mju_d2n

(** Convert from mjtNum to double. *)
let mju_n2d = Bindings.mju_n2d

(** Insertion sort, resulting list is in increasing order. *)
let mju_insertionSort = Bindings.mju_insertionSort

(** Integer insertion sort, resulting list is in increasing order. *)
let mju_insertionSortInt = Bindings.mju_insertionSortInt

(** Generate Halton sequence. *)
let mju_Halton = Bindings.mju_Halton

(** Call strncpy, then set dst[n-1] = 0. *)
let mju_strncpy = Bindings.mju_strncpy

(** Sigmoid function over 0<=x<=1 constructed from half-quadratics. *)
let mju_sigmoid = Bindings.mju_sigmoid
