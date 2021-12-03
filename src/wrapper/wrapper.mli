(* THIS FILE IS GENERATED AUTOMATICALLY, DO NOT EDIT BY HAND *)

open Stubs

type 'a t = 'a Ctypes.structure
type 'a ptr = 'a Ctypes_static.ptr

val ( !@ ) : 'a ptr -> 'a
val mj_addr : 'a t -> 'a t ptr

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
val mjtDisableBit_to_int : mjtDisableBit -> int

(** convert mjtEnableBit type to int *)
val mjtEnableBit_to_int : mjtEnableBit -> int

(** convert mjtJoint type to int *)
val mjtJoint_to_int : mjtJoint -> int

(** convert mjtGeom type to int *)
val mjtGeom_to_int : mjtGeom -> int

(** convert mjtCamLight type to int *)
val mjtCamLight_to_int : mjtCamLight -> int

(** convert mjtTexture type to int *)
val mjtTexture_to_int : mjtTexture -> int

(** convert mjtIntegrator type to int *)
val mjtIntegrator_to_int : mjtIntegrator -> int

(** convert mjtCollision type to int *)
val mjtCollision_to_int : mjtCollision -> int

(** convert mjtCone type to int *)
val mjtCone_to_int : mjtCone -> int

(** convert mjtJacobian type to int *)
val mjtJacobian_to_int : mjtJacobian -> int

(** convert mjtSolver type to int *)
val mjtSolver_to_int : mjtSolver -> int

(** convert mjtEq type to int *)
val mjtEq_to_int : mjtEq -> int

(** convert mjtWrap type to int *)
val mjtWrap_to_int : mjtWrap -> int

(** convert mjtTrn type to int *)
val mjtTrn_to_int : mjtTrn -> int

(** convert mjtDyn type to int *)
val mjtDyn_to_int : mjtDyn -> int

(** convert mjtGain type to int *)
val mjtGain_to_int : mjtGain -> int

(** convert mjtBias type to int *)
val mjtBias_to_int : mjtBias -> int

(** convert mjtObj type to int *)
val mjtObj_to_int : mjtObj -> int

(** convert mjtConstraint type to int *)
val mjtConstraint_to_int : mjtConstraint -> int

(** convert mjtConstraintState type to int *)
val mjtConstraintState_to_int : mjtConstraintState -> int

(** convert mjtSensor type to int *)
val mjtSensor_to_int : mjtSensor -> int

(** convert mjtStage type to int *)
val mjtStage_to_int : mjtStage -> int

(** convert mjtDataType type to int *)
val mjtDataType_to_int : mjtDataType -> int

(** convert mjtLRMode type to int *)
val mjtLRMode_to_int : mjtLRMode -> int

(** allocate fresh mjLROpt struct *)
val mjLROpt_allocate : unit -> mjLROpt

(** make null mjLROpt struct ptr *)
val mjLROpt_null : unit -> mjLROpt ptr

(** get mode from mjLROpt *)
val mjLROpt_get_mode : mjLROpt -> int

(** set mode for mjLROpt *)
val mjLROpt_set_mode : mjLROpt -> int -> unit

(** get useexisting from mjLROpt *)
val mjLROpt_get_useexisting : mjLROpt -> int

(** set useexisting for mjLROpt *)
val mjLROpt_set_useexisting : mjLROpt -> int -> unit

(** get uselimit from mjLROpt *)
val mjLROpt_get_uselimit : mjLROpt -> int

(** set uselimit for mjLROpt *)
val mjLROpt_set_uselimit : mjLROpt -> int -> unit

(** get accel from mjLROpt *)
val mjLROpt_get_accel : mjLROpt -> float

(** set accel for mjLROpt *)
val mjLROpt_set_accel : mjLROpt -> float -> unit

(** get maxforce from mjLROpt *)
val mjLROpt_get_maxforce : mjLROpt -> float

(** set maxforce for mjLROpt *)
val mjLROpt_set_maxforce : mjLROpt -> float -> unit

(** get timeconst from mjLROpt *)
val mjLROpt_get_timeconst : mjLROpt -> float

(** set timeconst for mjLROpt *)
val mjLROpt_set_timeconst : mjLROpt -> float -> unit

(** get timestep from mjLROpt *)
val mjLROpt_get_timestep : mjLROpt -> float

(** set timestep for mjLROpt *)
val mjLROpt_set_timestep : mjLROpt -> float -> unit

(** get inttotal from mjLROpt *)
val mjLROpt_get_inttotal : mjLROpt -> float

(** set inttotal for mjLROpt *)
val mjLROpt_set_inttotal : mjLROpt -> float -> unit

(** get inteval from mjLROpt *)
val mjLROpt_get_inteval : mjLROpt -> float

(** set inteval for mjLROpt *)
val mjLROpt_set_inteval : mjLROpt -> float -> unit

(** get tolrange from mjLROpt *)
val mjLROpt_get_tolrange : mjLROpt -> float

(** set tolrange for mjLROpt *)
val mjLROpt_set_tolrange : mjLROpt -> float -> unit

(** allocate fresh mjVFS struct *)
val mjVFS_allocate : unit -> mjVFS

(** make null mjVFS struct ptr *)
val mjVFS_null : unit -> mjVFS ptr

(** get nfile from mjVFS *)
val mjVFS_get_nfile : mjVFS -> int

(** set nfile for mjVFS *)
val mjVFS_set_nfile : mjVFS -> int -> unit

(** get filesize from mjVFS *)
val mjVFS_get_filesize : mjVFS -> int ptr

(** set filesize for mjVFS *)
val mjVFS_set_filesize : mjVFS -> int ptr -> unit

(** get filedata from mjVFS *)
val mjVFS_get_filedata : mjVFS -> unit ptr ptr

(** set filedata for mjVFS *)
val mjVFS_set_filedata : mjVFS -> unit ptr ptr -> unit

(** allocate fresh mjOption struct *)
val mjOption_allocate : unit -> mjOption

(** make null mjOption struct ptr *)
val mjOption_null : unit -> mjOption ptr

(** get timestep from mjOption *)
val mjOption_get_timestep : mjOption -> float

(** set timestep for mjOption *)
val mjOption_set_timestep : mjOption -> float -> unit

(** get apirate from mjOption *)
val mjOption_get_apirate : mjOption -> float

(** set apirate for mjOption *)
val mjOption_set_apirate : mjOption -> float -> unit

(** get impratio from mjOption *)
val mjOption_get_impratio : mjOption -> float

(** set impratio for mjOption *)
val mjOption_set_impratio : mjOption -> float -> unit

(** get tolerance from mjOption *)
val mjOption_get_tolerance : mjOption -> float

(** set tolerance for mjOption *)
val mjOption_set_tolerance : mjOption -> float -> unit

(** get noslip_tolerance from mjOption *)
val mjOption_get_noslip_tolerance : mjOption -> float

(** set noslip_tolerance for mjOption *)
val mjOption_set_noslip_tolerance : mjOption -> float -> unit

(** get mpr_tolerance from mjOption *)
val mjOption_get_mpr_tolerance : mjOption -> float

(** set mpr_tolerance for mjOption *)
val mjOption_set_mpr_tolerance : mjOption -> float -> unit

(** get gravity from mjOption *)
val mjOption_get_gravity : mjOption -> float ptr

(** set gravity for mjOption *)
val mjOption_set_gravity : mjOption -> float ptr -> unit

(** get wind from mjOption *)
val mjOption_get_wind : mjOption -> float ptr

(** set wind for mjOption *)
val mjOption_set_wind : mjOption -> float ptr -> unit

(** get magnetic from mjOption *)
val mjOption_get_magnetic : mjOption -> float ptr

(** set magnetic for mjOption *)
val mjOption_set_magnetic : mjOption -> float ptr -> unit

(** get density from mjOption *)
val mjOption_get_density : mjOption -> float

(** set density for mjOption *)
val mjOption_set_density : mjOption -> float -> unit

(** get viscosity from mjOption *)
val mjOption_get_viscosity : mjOption -> float

(** set viscosity for mjOption *)
val mjOption_set_viscosity : mjOption -> float -> unit

(** get o_margin from mjOption *)
val mjOption_get_o_margin : mjOption -> float

(** set o_margin for mjOption *)
val mjOption_set_o_margin : mjOption -> float -> unit

(** get o_solref from mjOption *)
val mjOption_get_o_solref : mjOption -> float ptr

(** set o_solref for mjOption *)
val mjOption_set_o_solref : mjOption -> float ptr -> unit

(** get o_solimp from mjOption *)
val mjOption_get_o_solimp : mjOption -> float ptr

(** set o_solimp for mjOption *)
val mjOption_set_o_solimp : mjOption -> float ptr -> unit

(** get integrator from mjOption *)
val mjOption_get_integrator : mjOption -> int

(** set integrator for mjOption *)
val mjOption_set_integrator : mjOption -> int -> unit

(** get collision from mjOption *)
val mjOption_get_collision : mjOption -> int

(** set collision for mjOption *)
val mjOption_set_collision : mjOption -> int -> unit

(** get cone from mjOption *)
val mjOption_get_cone : mjOption -> int

(** set cone for mjOption *)
val mjOption_set_cone : mjOption -> int -> unit

(** get jacobian from mjOption *)
val mjOption_get_jacobian : mjOption -> int

(** set jacobian for mjOption *)
val mjOption_set_jacobian : mjOption -> int -> unit

(** get solver from mjOption *)
val mjOption_get_solver : mjOption -> int

(** set solver for mjOption *)
val mjOption_set_solver : mjOption -> int -> unit

(** get iterations from mjOption *)
val mjOption_get_iterations : mjOption -> int

(** set iterations for mjOption *)
val mjOption_set_iterations : mjOption -> int -> unit

(** get noslip_iterations from mjOption *)
val mjOption_get_noslip_iterations : mjOption -> int

(** set noslip_iterations for mjOption *)
val mjOption_set_noslip_iterations : mjOption -> int -> unit

(** get mpr_iterations from mjOption *)
val mjOption_get_mpr_iterations : mjOption -> int

(** set mpr_iterations for mjOption *)
val mjOption_set_mpr_iterations : mjOption -> int -> unit

(** get disableflags from mjOption *)
val mjOption_get_disableflags : mjOption -> int

(** set disableflags for mjOption *)
val mjOption_set_disableflags : mjOption -> int -> unit

(** get enableflags from mjOption *)
val mjOption_get_enableflags : mjOption -> int

(** set enableflags for mjOption *)
val mjOption_set_enableflags : mjOption -> int -> unit

(** allocate fresh mjStatistic struct *)
val mjStatistic_allocate : unit -> mjStatistic

(** make null mjStatistic struct ptr *)
val mjStatistic_null : unit -> mjStatistic ptr

(** get meaninertia from mjStatistic *)
val mjStatistic_get_meaninertia : mjStatistic -> float

(** set meaninertia for mjStatistic *)
val mjStatistic_set_meaninertia : mjStatistic -> float -> unit

(** get meanmass from mjStatistic *)
val mjStatistic_get_meanmass : mjStatistic -> float

(** set meanmass for mjStatistic *)
val mjStatistic_set_meanmass : mjStatistic -> float -> unit

(** get meansize from mjStatistic *)
val mjStatistic_get_meansize : mjStatistic -> float

(** set meansize for mjStatistic *)
val mjStatistic_set_meansize : mjStatistic -> float -> unit

(** get extent from mjStatistic *)
val mjStatistic_get_extent : mjStatistic -> float

(** set extent for mjStatistic *)
val mjStatistic_set_extent : mjStatistic -> float -> unit

(** get center from mjStatistic *)
val mjStatistic_get_center : mjStatistic -> float ptr

(** set center for mjStatistic *)
val mjStatistic_set_center : mjStatistic -> float ptr -> unit

(** allocate fresh mjModel struct *)
val mjModel_allocate : unit -> mjModel

(** make null mjModel struct ptr *)
val mjModel_null : unit -> mjModel ptr

(** get nq from mjModel *)
val mjModel_get_nq : mjModel -> int

(** set nq for mjModel *)
val mjModel_set_nq : mjModel -> int -> unit

(** get nv from mjModel *)
val mjModel_get_nv : mjModel -> int

(** set nv for mjModel *)
val mjModel_set_nv : mjModel -> int -> unit

(** get nu from mjModel *)
val mjModel_get_nu : mjModel -> int

(** set nu for mjModel *)
val mjModel_set_nu : mjModel -> int -> unit

(** get na from mjModel *)
val mjModel_get_na : mjModel -> int

(** set na for mjModel *)
val mjModel_set_na : mjModel -> int -> unit

(** get nbody from mjModel *)
val mjModel_get_nbody : mjModel -> int

(** set nbody for mjModel *)
val mjModel_set_nbody : mjModel -> int -> unit

(** get njnt from mjModel *)
val mjModel_get_njnt : mjModel -> int

(** set njnt for mjModel *)
val mjModel_set_njnt : mjModel -> int -> unit

(** get ngeom from mjModel *)
val mjModel_get_ngeom : mjModel -> int

(** set ngeom for mjModel *)
val mjModel_set_ngeom : mjModel -> int -> unit

(** get nsite from mjModel *)
val mjModel_get_nsite : mjModel -> int

(** set nsite for mjModel *)
val mjModel_set_nsite : mjModel -> int -> unit

(** get ncam from mjModel *)
val mjModel_get_ncam : mjModel -> int

(** set ncam for mjModel *)
val mjModel_set_ncam : mjModel -> int -> unit

(** get nlight from mjModel *)
val mjModel_get_nlight : mjModel -> int

(** set nlight for mjModel *)
val mjModel_set_nlight : mjModel -> int -> unit

(** get nmesh from mjModel *)
val mjModel_get_nmesh : mjModel -> int

(** set nmesh for mjModel *)
val mjModel_set_nmesh : mjModel -> int -> unit

(** get nmeshvert from mjModel *)
val mjModel_get_nmeshvert : mjModel -> int

(** set nmeshvert for mjModel *)
val mjModel_set_nmeshvert : mjModel -> int -> unit

(** get nmeshtexvert from mjModel *)
val mjModel_get_nmeshtexvert : mjModel -> int

(** set nmeshtexvert for mjModel *)
val mjModel_set_nmeshtexvert : mjModel -> int -> unit

(** get nmeshface from mjModel *)
val mjModel_get_nmeshface : mjModel -> int

(** set nmeshface for mjModel *)
val mjModel_set_nmeshface : mjModel -> int -> unit

(** get nmeshgraph from mjModel *)
val mjModel_get_nmeshgraph : mjModel -> int

(** set nmeshgraph for mjModel *)
val mjModel_set_nmeshgraph : mjModel -> int -> unit

(** get nskin from mjModel *)
val mjModel_get_nskin : mjModel -> int

(** set nskin for mjModel *)
val mjModel_set_nskin : mjModel -> int -> unit

(** get nskinvert from mjModel *)
val mjModel_get_nskinvert : mjModel -> int

(** set nskinvert for mjModel *)
val mjModel_set_nskinvert : mjModel -> int -> unit

(** get nskintexvert from mjModel *)
val mjModel_get_nskintexvert : mjModel -> int

(** set nskintexvert for mjModel *)
val mjModel_set_nskintexvert : mjModel -> int -> unit

(** get nskinface from mjModel *)
val mjModel_get_nskinface : mjModel -> int

(** set nskinface for mjModel *)
val mjModel_set_nskinface : mjModel -> int -> unit

(** get nskinbone from mjModel *)
val mjModel_get_nskinbone : mjModel -> int

(** set nskinbone for mjModel *)
val mjModel_set_nskinbone : mjModel -> int -> unit

(** get nskinbonevert from mjModel *)
val mjModel_get_nskinbonevert : mjModel -> int

(** set nskinbonevert for mjModel *)
val mjModel_set_nskinbonevert : mjModel -> int -> unit

(** get nhfield from mjModel *)
val mjModel_get_nhfield : mjModel -> int

(** set nhfield for mjModel *)
val mjModel_set_nhfield : mjModel -> int -> unit

(** get nhfielddata from mjModel *)
val mjModel_get_nhfielddata : mjModel -> int

(** set nhfielddata for mjModel *)
val mjModel_set_nhfielddata : mjModel -> int -> unit

(** get ntex from mjModel *)
val mjModel_get_ntex : mjModel -> int

(** set ntex for mjModel *)
val mjModel_set_ntex : mjModel -> int -> unit

(** get ntexdata from mjModel *)
val mjModel_get_ntexdata : mjModel -> int

(** set ntexdata for mjModel *)
val mjModel_set_ntexdata : mjModel -> int -> unit

(** get nmat from mjModel *)
val mjModel_get_nmat : mjModel -> int

(** set nmat for mjModel *)
val mjModel_set_nmat : mjModel -> int -> unit

(** get npair from mjModel *)
val mjModel_get_npair : mjModel -> int

(** set npair for mjModel *)
val mjModel_set_npair : mjModel -> int -> unit

(** get nexclude from mjModel *)
val mjModel_get_nexclude : mjModel -> int

(** set nexclude for mjModel *)
val mjModel_set_nexclude : mjModel -> int -> unit

(** get neq from mjModel *)
val mjModel_get_neq : mjModel -> int

(** set neq for mjModel *)
val mjModel_set_neq : mjModel -> int -> unit

(** get ntendon from mjModel *)
val mjModel_get_ntendon : mjModel -> int

(** set ntendon for mjModel *)
val mjModel_set_ntendon : mjModel -> int -> unit

(** get nwrap from mjModel *)
val mjModel_get_nwrap : mjModel -> int

(** set nwrap for mjModel *)
val mjModel_set_nwrap : mjModel -> int -> unit

(** get nsensor from mjModel *)
val mjModel_get_nsensor : mjModel -> int

(** set nsensor for mjModel *)
val mjModel_set_nsensor : mjModel -> int -> unit

(** get nnumeric from mjModel *)
val mjModel_get_nnumeric : mjModel -> int

(** set nnumeric for mjModel *)
val mjModel_set_nnumeric : mjModel -> int -> unit

(** get nnumericdata from mjModel *)
val mjModel_get_nnumericdata : mjModel -> int

(** set nnumericdata for mjModel *)
val mjModel_set_nnumericdata : mjModel -> int -> unit

(** get ntext from mjModel *)
val mjModel_get_ntext : mjModel -> int

(** set ntext for mjModel *)
val mjModel_set_ntext : mjModel -> int -> unit

(** get ntextdata from mjModel *)
val mjModel_get_ntextdata : mjModel -> int

(** set ntextdata for mjModel *)
val mjModel_set_ntextdata : mjModel -> int -> unit

(** get ntuple from mjModel *)
val mjModel_get_ntuple : mjModel -> int

(** set ntuple for mjModel *)
val mjModel_set_ntuple : mjModel -> int -> unit

(** get ntupledata from mjModel *)
val mjModel_get_ntupledata : mjModel -> int

(** set ntupledata for mjModel *)
val mjModel_set_ntupledata : mjModel -> int -> unit

(** get nkey from mjModel *)
val mjModel_get_nkey : mjModel -> int

(** set nkey for mjModel *)
val mjModel_set_nkey : mjModel -> int -> unit

(** get nmocap from mjModel *)
val mjModel_get_nmocap : mjModel -> int

(** set nmocap for mjModel *)
val mjModel_set_nmocap : mjModel -> int -> unit

(** get nuser_body from mjModel *)
val mjModel_get_nuser_body : mjModel -> int

(** set nuser_body for mjModel *)
val mjModel_set_nuser_body : mjModel -> int -> unit

(** get nuser_jnt from mjModel *)
val mjModel_get_nuser_jnt : mjModel -> int

(** set nuser_jnt for mjModel *)
val mjModel_set_nuser_jnt : mjModel -> int -> unit

(** get nuser_geom from mjModel *)
val mjModel_get_nuser_geom : mjModel -> int

(** set nuser_geom for mjModel *)
val mjModel_set_nuser_geom : mjModel -> int -> unit

(** get nuser_site from mjModel *)
val mjModel_get_nuser_site : mjModel -> int

(** set nuser_site for mjModel *)
val mjModel_set_nuser_site : mjModel -> int -> unit

(** get nuser_cam from mjModel *)
val mjModel_get_nuser_cam : mjModel -> int

(** set nuser_cam for mjModel *)
val mjModel_set_nuser_cam : mjModel -> int -> unit

(** get nuser_tendon from mjModel *)
val mjModel_get_nuser_tendon : mjModel -> int

(** set nuser_tendon for mjModel *)
val mjModel_set_nuser_tendon : mjModel -> int -> unit

(** get nuser_actuator from mjModel *)
val mjModel_get_nuser_actuator : mjModel -> int

(** set nuser_actuator for mjModel *)
val mjModel_set_nuser_actuator : mjModel -> int -> unit

(** get nuser_sensor from mjModel *)
val mjModel_get_nuser_sensor : mjModel -> int

(** set nuser_sensor for mjModel *)
val mjModel_set_nuser_sensor : mjModel -> int -> unit

(** get nnames from mjModel *)
val mjModel_get_nnames : mjModel -> int

(** set nnames for mjModel *)
val mjModel_set_nnames : mjModel -> int -> unit

(** get nM from mjModel *)
val mjModel_get_nM : mjModel -> int

(** set nM for mjModel *)
val mjModel_set_nM : mjModel -> int -> unit

(** get nemax from mjModel *)
val mjModel_get_nemax : mjModel -> int

(** set nemax for mjModel *)
val mjModel_set_nemax : mjModel -> int -> unit

(** get njmax from mjModel *)
val mjModel_get_njmax : mjModel -> int

(** set njmax for mjModel *)
val mjModel_set_njmax : mjModel -> int -> unit

(** get nconmax from mjModel *)
val mjModel_get_nconmax : mjModel -> int

(** set nconmax for mjModel *)
val mjModel_set_nconmax : mjModel -> int -> unit

(** get nstack from mjModel *)
val mjModel_get_nstack : mjModel -> int

(** set nstack for mjModel *)
val mjModel_set_nstack : mjModel -> int -> unit

(** get nuserdata from mjModel *)
val mjModel_get_nuserdata : mjModel -> int

(** set nuserdata for mjModel *)
val mjModel_set_nuserdata : mjModel -> int -> unit

(** get nsensordata from mjModel *)
val mjModel_get_nsensordata : mjModel -> int

(** set nsensordata for mjModel *)
val mjModel_set_nsensordata : mjModel -> int -> unit

(** get nbuffer from mjModel *)
val mjModel_get_nbuffer : mjModel -> int

(** set nbuffer for mjModel *)
val mjModel_set_nbuffer : mjModel -> int -> unit

(** get opt from mjModel *)
val mjModel_get_opt : mjModel -> mjOption

(** set opt for mjModel *)
val mjModel_set_opt : mjModel -> mjOption -> unit

(** get vis from mjModel *)
val mjModel_get_vis : mjModel -> mjVisual

(** set vis for mjModel *)
val mjModel_set_vis : mjModel -> mjVisual -> unit

(** get stat from mjModel *)
val mjModel_get_stat : mjModel -> mjStatistic

(** set stat for mjModel *)
val mjModel_set_stat : mjModel -> mjStatistic -> unit

(** get buffer from mjModel *)
val mjModel_get_buffer : mjModel -> unit ptr

(** set buffer for mjModel *)
val mjModel_set_buffer : mjModel -> unit ptr -> unit

(** get qpos0 from mjModel *)
val mjModel_get_qpos0 : mjModel -> float ptr

(** set qpos0 for mjModel *)
val mjModel_set_qpos0 : mjModel -> float ptr -> unit

(** get qpos_spring from mjModel *)
val mjModel_get_qpos_spring : mjModel -> float ptr

(** set qpos_spring for mjModel *)
val mjModel_set_qpos_spring : mjModel -> float ptr -> unit

(** get body_parentid from mjModel *)
val mjModel_get_body_parentid : mjModel -> int ptr

(** set body_parentid for mjModel *)
val mjModel_set_body_parentid : mjModel -> int ptr -> unit

(** get body_rootid from mjModel *)
val mjModel_get_body_rootid : mjModel -> int ptr

(** set body_rootid for mjModel *)
val mjModel_set_body_rootid : mjModel -> int ptr -> unit

(** get body_weldid from mjModel *)
val mjModel_get_body_weldid : mjModel -> int ptr

(** set body_weldid for mjModel *)
val mjModel_set_body_weldid : mjModel -> int ptr -> unit

(** get body_mocapid from mjModel *)
val mjModel_get_body_mocapid : mjModel -> int ptr

(** set body_mocapid for mjModel *)
val mjModel_set_body_mocapid : mjModel -> int ptr -> unit

(** get body_jntnum from mjModel *)
val mjModel_get_body_jntnum : mjModel -> int ptr

(** set body_jntnum for mjModel *)
val mjModel_set_body_jntnum : mjModel -> int ptr -> unit

(** get body_jntadr from mjModel *)
val mjModel_get_body_jntadr : mjModel -> int ptr

(** set body_jntadr for mjModel *)
val mjModel_set_body_jntadr : mjModel -> int ptr -> unit

(** get body_dofnum from mjModel *)
val mjModel_get_body_dofnum : mjModel -> int ptr

(** set body_dofnum for mjModel *)
val mjModel_set_body_dofnum : mjModel -> int ptr -> unit

(** get body_dofadr from mjModel *)
val mjModel_get_body_dofadr : mjModel -> int ptr

(** set body_dofadr for mjModel *)
val mjModel_set_body_dofadr : mjModel -> int ptr -> unit

(** get body_geomnum from mjModel *)
val mjModel_get_body_geomnum : mjModel -> int ptr

(** set body_geomnum for mjModel *)
val mjModel_set_body_geomnum : mjModel -> int ptr -> unit

(** get body_geomadr from mjModel *)
val mjModel_get_body_geomadr : mjModel -> int ptr

(** set body_geomadr for mjModel *)
val mjModel_set_body_geomadr : mjModel -> int ptr -> unit

(** get body_simple from mjModel *)
val mjModel_get_body_simple : mjModel -> Unsigned.UChar.t ptr

(** set body_simple for mjModel *)
val mjModel_set_body_simple : mjModel -> Unsigned.UChar.t ptr -> unit

(** get body_sameframe from mjModel *)
val mjModel_get_body_sameframe : mjModel -> Unsigned.UChar.t ptr

(** set body_sameframe for mjModel *)
val mjModel_set_body_sameframe : mjModel -> Unsigned.UChar.t ptr -> unit

(** get body_pos from mjModel *)
val mjModel_get_body_pos : mjModel -> float ptr

(** set body_pos for mjModel *)
val mjModel_set_body_pos : mjModel -> float ptr -> unit

(** get body_quat from mjModel *)
val mjModel_get_body_quat : mjModel -> float ptr

(** set body_quat for mjModel *)
val mjModel_set_body_quat : mjModel -> float ptr -> unit

(** get body_ipos from mjModel *)
val mjModel_get_body_ipos : mjModel -> float ptr

(** set body_ipos for mjModel *)
val mjModel_set_body_ipos : mjModel -> float ptr -> unit

(** get body_iquat from mjModel *)
val mjModel_get_body_iquat : mjModel -> float ptr

(** set body_iquat for mjModel *)
val mjModel_set_body_iquat : mjModel -> float ptr -> unit

(** get body_mass from mjModel *)
val mjModel_get_body_mass : mjModel -> float ptr

(** set body_mass for mjModel *)
val mjModel_set_body_mass : mjModel -> float ptr -> unit

(** get body_subtreemass from mjModel *)
val mjModel_get_body_subtreemass : mjModel -> float ptr

(** set body_subtreemass for mjModel *)
val mjModel_set_body_subtreemass : mjModel -> float ptr -> unit

(** get body_inertia from mjModel *)
val mjModel_get_body_inertia : mjModel -> float ptr

(** set body_inertia for mjModel *)
val mjModel_set_body_inertia : mjModel -> float ptr -> unit

(** get body_invweight0 from mjModel *)
val mjModel_get_body_invweight0 : mjModel -> float ptr

(** set body_invweight0 for mjModel *)
val mjModel_set_body_invweight0 : mjModel -> float ptr -> unit

(** get body_user from mjModel *)
val mjModel_get_body_user : mjModel -> float ptr

(** set body_user for mjModel *)
val mjModel_set_body_user : mjModel -> float ptr -> unit

(** get jnt_type from mjModel *)
val mjModel_get_jnt_type : mjModel -> int ptr

(** set jnt_type for mjModel *)
val mjModel_set_jnt_type : mjModel -> int ptr -> unit

(** get jnt_qposadr from mjModel *)
val mjModel_get_jnt_qposadr : mjModel -> int ptr

(** set jnt_qposadr for mjModel *)
val mjModel_set_jnt_qposadr : mjModel -> int ptr -> unit

(** get jnt_dofadr from mjModel *)
val mjModel_get_jnt_dofadr : mjModel -> int ptr

(** set jnt_dofadr for mjModel *)
val mjModel_set_jnt_dofadr : mjModel -> int ptr -> unit

(** get jnt_bodyid from mjModel *)
val mjModel_get_jnt_bodyid : mjModel -> int ptr

(** set jnt_bodyid for mjModel *)
val mjModel_set_jnt_bodyid : mjModel -> int ptr -> unit

(** get jnt_group from mjModel *)
val mjModel_get_jnt_group : mjModel -> int ptr

(** set jnt_group for mjModel *)
val mjModel_set_jnt_group : mjModel -> int ptr -> unit

(** get jnt_limited from mjModel *)
val mjModel_get_jnt_limited : mjModel -> Unsigned.UChar.t ptr

(** set jnt_limited for mjModel *)
val mjModel_set_jnt_limited : mjModel -> Unsigned.UChar.t ptr -> unit

(** get jnt_solref from mjModel *)
val mjModel_get_jnt_solref : mjModel -> float ptr

(** set jnt_solref for mjModel *)
val mjModel_set_jnt_solref : mjModel -> float ptr -> unit

(** get jnt_solimp from mjModel *)
val mjModel_get_jnt_solimp : mjModel -> float ptr

(** set jnt_solimp for mjModel *)
val mjModel_set_jnt_solimp : mjModel -> float ptr -> unit

(** get jnt_pos from mjModel *)
val mjModel_get_jnt_pos : mjModel -> float ptr

(** set jnt_pos for mjModel *)
val mjModel_set_jnt_pos : mjModel -> float ptr -> unit

(** get jnt_axis from mjModel *)
val mjModel_get_jnt_axis : mjModel -> float ptr

(** set jnt_axis for mjModel *)
val mjModel_set_jnt_axis : mjModel -> float ptr -> unit

(** get jnt_stiffness from mjModel *)
val mjModel_get_jnt_stiffness : mjModel -> float ptr

(** set jnt_stiffness for mjModel *)
val mjModel_set_jnt_stiffness : mjModel -> float ptr -> unit

(** get jnt_range from mjModel *)
val mjModel_get_jnt_range : mjModel -> float ptr

(** set jnt_range for mjModel *)
val mjModel_set_jnt_range : mjModel -> float ptr -> unit

(** get jnt_margin from mjModel *)
val mjModel_get_jnt_margin : mjModel -> float ptr

(** set jnt_margin for mjModel *)
val mjModel_set_jnt_margin : mjModel -> float ptr -> unit

(** get jnt_user from mjModel *)
val mjModel_get_jnt_user : mjModel -> float ptr

(** set jnt_user for mjModel *)
val mjModel_set_jnt_user : mjModel -> float ptr -> unit

(** get dof_bodyid from mjModel *)
val mjModel_get_dof_bodyid : mjModel -> int ptr

(** set dof_bodyid for mjModel *)
val mjModel_set_dof_bodyid : mjModel -> int ptr -> unit

(** get dof_jntid from mjModel *)
val mjModel_get_dof_jntid : mjModel -> int ptr

(** set dof_jntid for mjModel *)
val mjModel_set_dof_jntid : mjModel -> int ptr -> unit

(** get dof_parentid from mjModel *)
val mjModel_get_dof_parentid : mjModel -> int ptr

(** set dof_parentid for mjModel *)
val mjModel_set_dof_parentid : mjModel -> int ptr -> unit

(** get dof_Madr from mjModel *)
val mjModel_get_dof_Madr : mjModel -> int ptr

(** set dof_Madr for mjModel *)
val mjModel_set_dof_Madr : mjModel -> int ptr -> unit

(** get dof_simplenum from mjModel *)
val mjModel_get_dof_simplenum : mjModel -> int ptr

(** set dof_simplenum for mjModel *)
val mjModel_set_dof_simplenum : mjModel -> int ptr -> unit

(** get dof_solref from mjModel *)
val mjModel_get_dof_solref : mjModel -> float ptr

(** set dof_solref for mjModel *)
val mjModel_set_dof_solref : mjModel -> float ptr -> unit

(** get dof_solimp from mjModel *)
val mjModel_get_dof_solimp : mjModel -> float ptr

(** set dof_solimp for mjModel *)
val mjModel_set_dof_solimp : mjModel -> float ptr -> unit

(** get dof_frictionloss from mjModel *)
val mjModel_get_dof_frictionloss : mjModel -> float ptr

(** set dof_frictionloss for mjModel *)
val mjModel_set_dof_frictionloss : mjModel -> float ptr -> unit

(** get dof_armature from mjModel *)
val mjModel_get_dof_armature : mjModel -> float ptr

(** set dof_armature for mjModel *)
val mjModel_set_dof_armature : mjModel -> float ptr -> unit

(** get dof_damping from mjModel *)
val mjModel_get_dof_damping : mjModel -> float ptr

(** set dof_damping for mjModel *)
val mjModel_set_dof_damping : mjModel -> float ptr -> unit

(** get dof_invweight0 from mjModel *)
val mjModel_get_dof_invweight0 : mjModel -> float ptr

(** set dof_invweight0 for mjModel *)
val mjModel_set_dof_invweight0 : mjModel -> float ptr -> unit

(** get dof_M0 from mjModel *)
val mjModel_get_dof_M0 : mjModel -> float ptr

(** set dof_M0 for mjModel *)
val mjModel_set_dof_M0 : mjModel -> float ptr -> unit

(** get geom_type from mjModel *)
val mjModel_get_geom_type : mjModel -> int ptr

(** set geom_type for mjModel *)
val mjModel_set_geom_type : mjModel -> int ptr -> unit

(** get geom_contype from mjModel *)
val mjModel_get_geom_contype : mjModel -> int ptr

(** set geom_contype for mjModel *)
val mjModel_set_geom_contype : mjModel -> int ptr -> unit

(** get geom_conaffinity from mjModel *)
val mjModel_get_geom_conaffinity : mjModel -> int ptr

(** set geom_conaffinity for mjModel *)
val mjModel_set_geom_conaffinity : mjModel -> int ptr -> unit

(** get geom_condim from mjModel *)
val mjModel_get_geom_condim : mjModel -> int ptr

(** set geom_condim for mjModel *)
val mjModel_set_geom_condim : mjModel -> int ptr -> unit

(** get geom_bodyid from mjModel *)
val mjModel_get_geom_bodyid : mjModel -> int ptr

(** set geom_bodyid for mjModel *)
val mjModel_set_geom_bodyid : mjModel -> int ptr -> unit

(** get geom_dataid from mjModel *)
val mjModel_get_geom_dataid : mjModel -> int ptr

(** set geom_dataid for mjModel *)
val mjModel_set_geom_dataid : mjModel -> int ptr -> unit

(** get geom_matid from mjModel *)
val mjModel_get_geom_matid : mjModel -> int ptr

(** set geom_matid for mjModel *)
val mjModel_set_geom_matid : mjModel -> int ptr -> unit

(** get geom_group from mjModel *)
val mjModel_get_geom_group : mjModel -> int ptr

(** set geom_group for mjModel *)
val mjModel_set_geom_group : mjModel -> int ptr -> unit

(** get geom_priority from mjModel *)
val mjModel_get_geom_priority : mjModel -> int ptr

(** set geom_priority for mjModel *)
val mjModel_set_geom_priority : mjModel -> int ptr -> unit

(** get geom_sameframe from mjModel *)
val mjModel_get_geom_sameframe : mjModel -> Unsigned.UChar.t ptr

(** set geom_sameframe for mjModel *)
val mjModel_set_geom_sameframe : mjModel -> Unsigned.UChar.t ptr -> unit

(** get geom_solmix from mjModel *)
val mjModel_get_geom_solmix : mjModel -> float ptr

(** set geom_solmix for mjModel *)
val mjModel_set_geom_solmix : mjModel -> float ptr -> unit

(** get geom_solref from mjModel *)
val mjModel_get_geom_solref : mjModel -> float ptr

(** set geom_solref for mjModel *)
val mjModel_set_geom_solref : mjModel -> float ptr -> unit

(** get geom_solimp from mjModel *)
val mjModel_get_geom_solimp : mjModel -> float ptr

(** set geom_solimp for mjModel *)
val mjModel_set_geom_solimp : mjModel -> float ptr -> unit

(** get geom_size from mjModel *)
val mjModel_get_geom_size : mjModel -> float ptr

(** set geom_size for mjModel *)
val mjModel_set_geom_size : mjModel -> float ptr -> unit

(** get geom_rbound from mjModel *)
val mjModel_get_geom_rbound : mjModel -> float ptr

(** set geom_rbound for mjModel *)
val mjModel_set_geom_rbound : mjModel -> float ptr -> unit

(** get geom_pos from mjModel *)
val mjModel_get_geom_pos : mjModel -> float ptr

(** set geom_pos for mjModel *)
val mjModel_set_geom_pos : mjModel -> float ptr -> unit

(** get geom_quat from mjModel *)
val mjModel_get_geom_quat : mjModel -> float ptr

(** set geom_quat for mjModel *)
val mjModel_set_geom_quat : mjModel -> float ptr -> unit

(** get geom_friction from mjModel *)
val mjModel_get_geom_friction : mjModel -> float ptr

(** set geom_friction for mjModel *)
val mjModel_set_geom_friction : mjModel -> float ptr -> unit

(** get geom_margin from mjModel *)
val mjModel_get_geom_margin : mjModel -> float ptr

(** set geom_margin for mjModel *)
val mjModel_set_geom_margin : mjModel -> float ptr -> unit

(** get geom_gap from mjModel *)
val mjModel_get_geom_gap : mjModel -> float ptr

(** set geom_gap for mjModel *)
val mjModel_set_geom_gap : mjModel -> float ptr -> unit

(** get geom_user from mjModel *)
val mjModel_get_geom_user : mjModel -> float ptr

(** set geom_user for mjModel *)
val mjModel_set_geom_user : mjModel -> float ptr -> unit

(** get geom_rgba from mjModel *)
val mjModel_get_geom_rgba : mjModel -> float ptr

(** set geom_rgba for mjModel *)
val mjModel_set_geom_rgba : mjModel -> float ptr -> unit

(** get site_type from mjModel *)
val mjModel_get_site_type : mjModel -> int ptr

(** set site_type for mjModel *)
val mjModel_set_site_type : mjModel -> int ptr -> unit

(** get site_bodyid from mjModel *)
val mjModel_get_site_bodyid : mjModel -> int ptr

(** set site_bodyid for mjModel *)
val mjModel_set_site_bodyid : mjModel -> int ptr -> unit

(** get site_matid from mjModel *)
val mjModel_get_site_matid : mjModel -> int ptr

(** set site_matid for mjModel *)
val mjModel_set_site_matid : mjModel -> int ptr -> unit

(** get site_group from mjModel *)
val mjModel_get_site_group : mjModel -> int ptr

(** set site_group for mjModel *)
val mjModel_set_site_group : mjModel -> int ptr -> unit

(** get site_sameframe from mjModel *)
val mjModel_get_site_sameframe : mjModel -> Unsigned.UChar.t ptr

(** set site_sameframe for mjModel *)
val mjModel_set_site_sameframe : mjModel -> Unsigned.UChar.t ptr -> unit

(** get site_size from mjModel *)
val mjModel_get_site_size : mjModel -> float ptr

(** set site_size for mjModel *)
val mjModel_set_site_size : mjModel -> float ptr -> unit

(** get site_pos from mjModel *)
val mjModel_get_site_pos : mjModel -> float ptr

(** set site_pos for mjModel *)
val mjModel_set_site_pos : mjModel -> float ptr -> unit

(** get site_quat from mjModel *)
val mjModel_get_site_quat : mjModel -> float ptr

(** set site_quat for mjModel *)
val mjModel_set_site_quat : mjModel -> float ptr -> unit

(** get site_user from mjModel *)
val mjModel_get_site_user : mjModel -> float ptr

(** set site_user for mjModel *)
val mjModel_set_site_user : mjModel -> float ptr -> unit

(** get site_rgba from mjModel *)
val mjModel_get_site_rgba : mjModel -> float ptr

(** set site_rgba for mjModel *)
val mjModel_set_site_rgba : mjModel -> float ptr -> unit

(** get cam_mode from mjModel *)
val mjModel_get_cam_mode : mjModel -> int ptr

(** set cam_mode for mjModel *)
val mjModel_set_cam_mode : mjModel -> int ptr -> unit

(** get cam_bodyid from mjModel *)
val mjModel_get_cam_bodyid : mjModel -> int ptr

(** set cam_bodyid for mjModel *)
val mjModel_set_cam_bodyid : mjModel -> int ptr -> unit

(** get cam_targetbodyid from mjModel *)
val mjModel_get_cam_targetbodyid : mjModel -> int ptr

(** set cam_targetbodyid for mjModel *)
val mjModel_set_cam_targetbodyid : mjModel -> int ptr -> unit

(** get cam_pos from mjModel *)
val mjModel_get_cam_pos : mjModel -> float ptr

(** set cam_pos for mjModel *)
val mjModel_set_cam_pos : mjModel -> float ptr -> unit

(** get cam_quat from mjModel *)
val mjModel_get_cam_quat : mjModel -> float ptr

(** set cam_quat for mjModel *)
val mjModel_set_cam_quat : mjModel -> float ptr -> unit

(** get cam_poscom0 from mjModel *)
val mjModel_get_cam_poscom0 : mjModel -> float ptr

(** set cam_poscom0 for mjModel *)
val mjModel_set_cam_poscom0 : mjModel -> float ptr -> unit

(** get cam_pos0 from mjModel *)
val mjModel_get_cam_pos0 : mjModel -> float ptr

(** set cam_pos0 for mjModel *)
val mjModel_set_cam_pos0 : mjModel -> float ptr -> unit

(** get cam_mat0 from mjModel *)
val mjModel_get_cam_mat0 : mjModel -> float ptr

(** set cam_mat0 for mjModel *)
val mjModel_set_cam_mat0 : mjModel -> float ptr -> unit

(** get cam_fovy from mjModel *)
val mjModel_get_cam_fovy : mjModel -> float ptr

(** set cam_fovy for mjModel *)
val mjModel_set_cam_fovy : mjModel -> float ptr -> unit

(** get cam_ipd from mjModel *)
val mjModel_get_cam_ipd : mjModel -> float ptr

(** set cam_ipd for mjModel *)
val mjModel_set_cam_ipd : mjModel -> float ptr -> unit

(** get cam_user from mjModel *)
val mjModel_get_cam_user : mjModel -> float ptr

(** set cam_user for mjModel *)
val mjModel_set_cam_user : mjModel -> float ptr -> unit

(** get light_mode from mjModel *)
val mjModel_get_light_mode : mjModel -> int ptr

(** set light_mode for mjModel *)
val mjModel_set_light_mode : mjModel -> int ptr -> unit

(** get light_bodyid from mjModel *)
val mjModel_get_light_bodyid : mjModel -> int ptr

(** set light_bodyid for mjModel *)
val mjModel_set_light_bodyid : mjModel -> int ptr -> unit

(** get light_targetbodyid from mjModel *)
val mjModel_get_light_targetbodyid : mjModel -> int ptr

(** set light_targetbodyid for mjModel *)
val mjModel_set_light_targetbodyid : mjModel -> int ptr -> unit

(** get light_directional from mjModel *)
val mjModel_get_light_directional : mjModel -> Unsigned.UChar.t ptr

(** set light_directional for mjModel *)
val mjModel_set_light_directional : mjModel -> Unsigned.UChar.t ptr -> unit

(** get light_castshadow from mjModel *)
val mjModel_get_light_castshadow : mjModel -> Unsigned.UChar.t ptr

(** set light_castshadow for mjModel *)
val mjModel_set_light_castshadow : mjModel -> Unsigned.UChar.t ptr -> unit

(** get light_active from mjModel *)
val mjModel_get_light_active : mjModel -> Unsigned.UChar.t ptr

(** set light_active for mjModel *)
val mjModel_set_light_active : mjModel -> Unsigned.UChar.t ptr -> unit

(** get light_pos from mjModel *)
val mjModel_get_light_pos : mjModel -> float ptr

(** set light_pos for mjModel *)
val mjModel_set_light_pos : mjModel -> float ptr -> unit

(** get light_dir from mjModel *)
val mjModel_get_light_dir : mjModel -> float ptr

(** set light_dir for mjModel *)
val mjModel_set_light_dir : mjModel -> float ptr -> unit

(** get light_poscom0 from mjModel *)
val mjModel_get_light_poscom0 : mjModel -> float ptr

(** set light_poscom0 for mjModel *)
val mjModel_set_light_poscom0 : mjModel -> float ptr -> unit

(** get light_pos0 from mjModel *)
val mjModel_get_light_pos0 : mjModel -> float ptr

(** set light_pos0 for mjModel *)
val mjModel_set_light_pos0 : mjModel -> float ptr -> unit

(** get light_dir0 from mjModel *)
val mjModel_get_light_dir0 : mjModel -> float ptr

(** set light_dir0 for mjModel *)
val mjModel_set_light_dir0 : mjModel -> float ptr -> unit

(** get light_attenuation from mjModel *)
val mjModel_get_light_attenuation : mjModel -> float ptr

(** set light_attenuation for mjModel *)
val mjModel_set_light_attenuation : mjModel -> float ptr -> unit

(** get light_cutoff from mjModel *)
val mjModel_get_light_cutoff : mjModel -> float ptr

(** set light_cutoff for mjModel *)
val mjModel_set_light_cutoff : mjModel -> float ptr -> unit

(** get light_exponent from mjModel *)
val mjModel_get_light_exponent : mjModel -> float ptr

(** set light_exponent for mjModel *)
val mjModel_set_light_exponent : mjModel -> float ptr -> unit

(** get light_ambient from mjModel *)
val mjModel_get_light_ambient : mjModel -> float ptr

(** set light_ambient for mjModel *)
val mjModel_set_light_ambient : mjModel -> float ptr -> unit

(** get light_diffuse from mjModel *)
val mjModel_get_light_diffuse : mjModel -> float ptr

(** set light_diffuse for mjModel *)
val mjModel_set_light_diffuse : mjModel -> float ptr -> unit

(** get light_specular from mjModel *)
val mjModel_get_light_specular : mjModel -> float ptr

(** set light_specular for mjModel *)
val mjModel_set_light_specular : mjModel -> float ptr -> unit

(** get mesh_vertadr from mjModel *)
val mjModel_get_mesh_vertadr : mjModel -> int ptr

(** set mesh_vertadr for mjModel *)
val mjModel_set_mesh_vertadr : mjModel -> int ptr -> unit

(** get mesh_vertnum from mjModel *)
val mjModel_get_mesh_vertnum : mjModel -> int ptr

(** set mesh_vertnum for mjModel *)
val mjModel_set_mesh_vertnum : mjModel -> int ptr -> unit

(** get mesh_texcoordadr from mjModel *)
val mjModel_get_mesh_texcoordadr : mjModel -> int ptr

(** set mesh_texcoordadr for mjModel *)
val mjModel_set_mesh_texcoordadr : mjModel -> int ptr -> unit

(** get mesh_faceadr from mjModel *)
val mjModel_get_mesh_faceadr : mjModel -> int ptr

(** set mesh_faceadr for mjModel *)
val mjModel_set_mesh_faceadr : mjModel -> int ptr -> unit

(** get mesh_facenum from mjModel *)
val mjModel_get_mesh_facenum : mjModel -> int ptr

(** set mesh_facenum for mjModel *)
val mjModel_set_mesh_facenum : mjModel -> int ptr -> unit

(** get mesh_graphadr from mjModel *)
val mjModel_get_mesh_graphadr : mjModel -> int ptr

(** set mesh_graphadr for mjModel *)
val mjModel_set_mesh_graphadr : mjModel -> int ptr -> unit

(** get mesh_vert from mjModel *)
val mjModel_get_mesh_vert : mjModel -> float ptr

(** set mesh_vert for mjModel *)
val mjModel_set_mesh_vert : mjModel -> float ptr -> unit

(** get mesh_normal from mjModel *)
val mjModel_get_mesh_normal : mjModel -> float ptr

(** set mesh_normal for mjModel *)
val mjModel_set_mesh_normal : mjModel -> float ptr -> unit

(** get mesh_texcoord from mjModel *)
val mjModel_get_mesh_texcoord : mjModel -> float ptr

(** set mesh_texcoord for mjModel *)
val mjModel_set_mesh_texcoord : mjModel -> float ptr -> unit

(** get mesh_face from mjModel *)
val mjModel_get_mesh_face : mjModel -> int ptr

(** set mesh_face for mjModel *)
val mjModel_set_mesh_face : mjModel -> int ptr -> unit

(** get mesh_graph from mjModel *)
val mjModel_get_mesh_graph : mjModel -> int ptr

(** set mesh_graph for mjModel *)
val mjModel_set_mesh_graph : mjModel -> int ptr -> unit

(** get skin_matid from mjModel *)
val mjModel_get_skin_matid : mjModel -> int ptr

(** set skin_matid for mjModel *)
val mjModel_set_skin_matid : mjModel -> int ptr -> unit

(** get skin_rgba from mjModel *)
val mjModel_get_skin_rgba : mjModel -> float ptr

(** set skin_rgba for mjModel *)
val mjModel_set_skin_rgba : mjModel -> float ptr -> unit

(** get skin_inflate from mjModel *)
val mjModel_get_skin_inflate : mjModel -> float ptr

(** set skin_inflate for mjModel *)
val mjModel_set_skin_inflate : mjModel -> float ptr -> unit

(** get skin_vertadr from mjModel *)
val mjModel_get_skin_vertadr : mjModel -> int ptr

(** set skin_vertadr for mjModel *)
val mjModel_set_skin_vertadr : mjModel -> int ptr -> unit

(** get skin_vertnum from mjModel *)
val mjModel_get_skin_vertnum : mjModel -> int ptr

(** set skin_vertnum for mjModel *)
val mjModel_set_skin_vertnum : mjModel -> int ptr -> unit

(** get skin_texcoordadr from mjModel *)
val mjModel_get_skin_texcoordadr : mjModel -> int ptr

(** set skin_texcoordadr for mjModel *)
val mjModel_set_skin_texcoordadr : mjModel -> int ptr -> unit

(** get skin_faceadr from mjModel *)
val mjModel_get_skin_faceadr : mjModel -> int ptr

(** set skin_faceadr for mjModel *)
val mjModel_set_skin_faceadr : mjModel -> int ptr -> unit

(** get skin_facenum from mjModel *)
val mjModel_get_skin_facenum : mjModel -> int ptr

(** set skin_facenum for mjModel *)
val mjModel_set_skin_facenum : mjModel -> int ptr -> unit

(** get skin_boneadr from mjModel *)
val mjModel_get_skin_boneadr : mjModel -> int ptr

(** set skin_boneadr for mjModel *)
val mjModel_set_skin_boneadr : mjModel -> int ptr -> unit

(** get skin_bonenum from mjModel *)
val mjModel_get_skin_bonenum : mjModel -> int ptr

(** set skin_bonenum for mjModel *)
val mjModel_set_skin_bonenum : mjModel -> int ptr -> unit

(** get skin_vert from mjModel *)
val mjModel_get_skin_vert : mjModel -> float ptr

(** set skin_vert for mjModel *)
val mjModel_set_skin_vert : mjModel -> float ptr -> unit

(** get skin_texcoord from mjModel *)
val mjModel_get_skin_texcoord : mjModel -> float ptr

(** set skin_texcoord for mjModel *)
val mjModel_set_skin_texcoord : mjModel -> float ptr -> unit

(** get skin_face from mjModel *)
val mjModel_get_skin_face : mjModel -> int ptr

(** set skin_face for mjModel *)
val mjModel_set_skin_face : mjModel -> int ptr -> unit

(** get skin_bonevertadr from mjModel *)
val mjModel_get_skin_bonevertadr : mjModel -> int ptr

(** set skin_bonevertadr for mjModel *)
val mjModel_set_skin_bonevertadr : mjModel -> int ptr -> unit

(** get skin_bonevertnum from mjModel *)
val mjModel_get_skin_bonevertnum : mjModel -> int ptr

(** set skin_bonevertnum for mjModel *)
val mjModel_set_skin_bonevertnum : mjModel -> int ptr -> unit

(** get skin_bonebindpos from mjModel *)
val mjModel_get_skin_bonebindpos : mjModel -> float ptr

(** set skin_bonebindpos for mjModel *)
val mjModel_set_skin_bonebindpos : mjModel -> float ptr -> unit

(** get skin_bonebindquat from mjModel *)
val mjModel_get_skin_bonebindquat : mjModel -> float ptr

(** set skin_bonebindquat for mjModel *)
val mjModel_set_skin_bonebindquat : mjModel -> float ptr -> unit

(** get skin_bonebodyid from mjModel *)
val mjModel_get_skin_bonebodyid : mjModel -> int ptr

(** set skin_bonebodyid for mjModel *)
val mjModel_set_skin_bonebodyid : mjModel -> int ptr -> unit

(** get skin_bonevertid from mjModel *)
val mjModel_get_skin_bonevertid : mjModel -> int ptr

(** set skin_bonevertid for mjModel *)
val mjModel_set_skin_bonevertid : mjModel -> int ptr -> unit

(** get skin_bonevertweight from mjModel *)
val mjModel_get_skin_bonevertweight : mjModel -> float ptr

(** set skin_bonevertweight for mjModel *)
val mjModel_set_skin_bonevertweight : mjModel -> float ptr -> unit

(** get hfield_size from mjModel *)
val mjModel_get_hfield_size : mjModel -> float ptr

(** set hfield_size for mjModel *)
val mjModel_set_hfield_size : mjModel -> float ptr -> unit

(** get hfield_nrow from mjModel *)
val mjModel_get_hfield_nrow : mjModel -> int ptr

(** set hfield_nrow for mjModel *)
val mjModel_set_hfield_nrow : mjModel -> int ptr -> unit

(** get hfield_ncol from mjModel *)
val mjModel_get_hfield_ncol : mjModel -> int ptr

(** set hfield_ncol for mjModel *)
val mjModel_set_hfield_ncol : mjModel -> int ptr -> unit

(** get hfield_adr from mjModel *)
val mjModel_get_hfield_adr : mjModel -> int ptr

(** set hfield_adr for mjModel *)
val mjModel_set_hfield_adr : mjModel -> int ptr -> unit

(** get hfield_data from mjModel *)
val mjModel_get_hfield_data : mjModel -> float ptr

(** set hfield_data for mjModel *)
val mjModel_set_hfield_data : mjModel -> float ptr -> unit

(** get tex_type from mjModel *)
val mjModel_get_tex_type : mjModel -> int ptr

(** set tex_type for mjModel *)
val mjModel_set_tex_type : mjModel -> int ptr -> unit

(** get tex_height from mjModel *)
val mjModel_get_tex_height : mjModel -> int ptr

(** set tex_height for mjModel *)
val mjModel_set_tex_height : mjModel -> int ptr -> unit

(** get tex_width from mjModel *)
val mjModel_get_tex_width : mjModel -> int ptr

(** set tex_width for mjModel *)
val mjModel_set_tex_width : mjModel -> int ptr -> unit

(** get tex_adr from mjModel *)
val mjModel_get_tex_adr : mjModel -> int ptr

(** set tex_adr for mjModel *)
val mjModel_set_tex_adr : mjModel -> int ptr -> unit

(** get tex_rgb from mjModel *)
val mjModel_get_tex_rgb : mjModel -> Unsigned.UChar.t ptr

(** set tex_rgb for mjModel *)
val mjModel_set_tex_rgb : mjModel -> Unsigned.UChar.t ptr -> unit

(** get mat_texid from mjModel *)
val mjModel_get_mat_texid : mjModel -> int ptr

(** set mat_texid for mjModel *)
val mjModel_set_mat_texid : mjModel -> int ptr -> unit

(** get mat_texuniform from mjModel *)
val mjModel_get_mat_texuniform : mjModel -> Unsigned.UChar.t ptr

(** set mat_texuniform for mjModel *)
val mjModel_set_mat_texuniform : mjModel -> Unsigned.UChar.t ptr -> unit

(** get mat_texrepeat from mjModel *)
val mjModel_get_mat_texrepeat : mjModel -> float ptr

(** set mat_texrepeat for mjModel *)
val mjModel_set_mat_texrepeat : mjModel -> float ptr -> unit

(** get mat_emission from mjModel *)
val mjModel_get_mat_emission : mjModel -> float ptr

(** set mat_emission for mjModel *)
val mjModel_set_mat_emission : mjModel -> float ptr -> unit

(** get mat_specular from mjModel *)
val mjModel_get_mat_specular : mjModel -> float ptr

(** set mat_specular for mjModel *)
val mjModel_set_mat_specular : mjModel -> float ptr -> unit

(** get mat_shininess from mjModel *)
val mjModel_get_mat_shininess : mjModel -> float ptr

(** set mat_shininess for mjModel *)
val mjModel_set_mat_shininess : mjModel -> float ptr -> unit

(** get mat_reflectance from mjModel *)
val mjModel_get_mat_reflectance : mjModel -> float ptr

(** set mat_reflectance for mjModel *)
val mjModel_set_mat_reflectance : mjModel -> float ptr -> unit

(** get mat_rgba from mjModel *)
val mjModel_get_mat_rgba : mjModel -> float ptr

(** set mat_rgba for mjModel *)
val mjModel_set_mat_rgba : mjModel -> float ptr -> unit

(** get pair_dim from mjModel *)
val mjModel_get_pair_dim : mjModel -> int ptr

(** set pair_dim for mjModel *)
val mjModel_set_pair_dim : mjModel -> int ptr -> unit

(** get pair_geom1 from mjModel *)
val mjModel_get_pair_geom1 : mjModel -> int ptr

(** set pair_geom1 for mjModel *)
val mjModel_set_pair_geom1 : mjModel -> int ptr -> unit

(** get pair_geom2 from mjModel *)
val mjModel_get_pair_geom2 : mjModel -> int ptr

(** set pair_geom2 for mjModel *)
val mjModel_set_pair_geom2 : mjModel -> int ptr -> unit

(** get pair_signature from mjModel *)
val mjModel_get_pair_signature : mjModel -> int ptr

(** set pair_signature for mjModel *)
val mjModel_set_pair_signature : mjModel -> int ptr -> unit

(** get pair_solref from mjModel *)
val mjModel_get_pair_solref : mjModel -> float ptr

(** set pair_solref for mjModel *)
val mjModel_set_pair_solref : mjModel -> float ptr -> unit

(** get pair_solimp from mjModel *)
val mjModel_get_pair_solimp : mjModel -> float ptr

(** set pair_solimp for mjModel *)
val mjModel_set_pair_solimp : mjModel -> float ptr -> unit

(** get pair_margin from mjModel *)
val mjModel_get_pair_margin : mjModel -> float ptr

(** set pair_margin for mjModel *)
val mjModel_set_pair_margin : mjModel -> float ptr -> unit

(** get pair_gap from mjModel *)
val mjModel_get_pair_gap : mjModel -> float ptr

(** set pair_gap for mjModel *)
val mjModel_set_pair_gap : mjModel -> float ptr -> unit

(** get pair_friction from mjModel *)
val mjModel_get_pair_friction : mjModel -> float ptr

(** set pair_friction for mjModel *)
val mjModel_set_pair_friction : mjModel -> float ptr -> unit

(** get exclude_signature from mjModel *)
val mjModel_get_exclude_signature : mjModel -> int ptr

(** set exclude_signature for mjModel *)
val mjModel_set_exclude_signature : mjModel -> int ptr -> unit

(** get eq_type from mjModel *)
val mjModel_get_eq_type : mjModel -> int ptr

(** set eq_type for mjModel *)
val mjModel_set_eq_type : mjModel -> int ptr -> unit

(** get eq_obj1id from mjModel *)
val mjModel_get_eq_obj1id : mjModel -> int ptr

(** set eq_obj1id for mjModel *)
val mjModel_set_eq_obj1id : mjModel -> int ptr -> unit

(** get eq_obj2id from mjModel *)
val mjModel_get_eq_obj2id : mjModel -> int ptr

(** set eq_obj2id for mjModel *)
val mjModel_set_eq_obj2id : mjModel -> int ptr -> unit

(** get eq_active from mjModel *)
val mjModel_get_eq_active : mjModel -> Unsigned.UChar.t ptr

(** set eq_active for mjModel *)
val mjModel_set_eq_active : mjModel -> Unsigned.UChar.t ptr -> unit

(** get eq_solref from mjModel *)
val mjModel_get_eq_solref : mjModel -> float ptr

(** set eq_solref for mjModel *)
val mjModel_set_eq_solref : mjModel -> float ptr -> unit

(** get eq_solimp from mjModel *)
val mjModel_get_eq_solimp : mjModel -> float ptr

(** set eq_solimp for mjModel *)
val mjModel_set_eq_solimp : mjModel -> float ptr -> unit

(** get eq_data from mjModel *)
val mjModel_get_eq_data : mjModel -> float ptr

(** set eq_data for mjModel *)
val mjModel_set_eq_data : mjModel -> float ptr -> unit

(** get tendon_adr from mjModel *)
val mjModel_get_tendon_adr : mjModel -> int ptr

(** set tendon_adr for mjModel *)
val mjModel_set_tendon_adr : mjModel -> int ptr -> unit

(** get tendon_num from mjModel *)
val mjModel_get_tendon_num : mjModel -> int ptr

(** set tendon_num for mjModel *)
val mjModel_set_tendon_num : mjModel -> int ptr -> unit

(** get tendon_matid from mjModel *)
val mjModel_get_tendon_matid : mjModel -> int ptr

(** set tendon_matid for mjModel *)
val mjModel_set_tendon_matid : mjModel -> int ptr -> unit

(** get tendon_group from mjModel *)
val mjModel_get_tendon_group : mjModel -> int ptr

(** set tendon_group for mjModel *)
val mjModel_set_tendon_group : mjModel -> int ptr -> unit

(** get tendon_limited from mjModel *)
val mjModel_get_tendon_limited : mjModel -> Unsigned.UChar.t ptr

(** set tendon_limited for mjModel *)
val mjModel_set_tendon_limited : mjModel -> Unsigned.UChar.t ptr -> unit

(** get tendon_width from mjModel *)
val mjModel_get_tendon_width : mjModel -> float ptr

(** set tendon_width for mjModel *)
val mjModel_set_tendon_width : mjModel -> float ptr -> unit

(** get tendon_solref_lim from mjModel *)
val mjModel_get_tendon_solref_lim : mjModel -> float ptr

(** set tendon_solref_lim for mjModel *)
val mjModel_set_tendon_solref_lim : mjModel -> float ptr -> unit

(** get tendon_solimp_lim from mjModel *)
val mjModel_get_tendon_solimp_lim : mjModel -> float ptr

(** set tendon_solimp_lim for mjModel *)
val mjModel_set_tendon_solimp_lim : mjModel -> float ptr -> unit

(** get tendon_solref_fri from mjModel *)
val mjModel_get_tendon_solref_fri : mjModel -> float ptr

(** set tendon_solref_fri for mjModel *)
val mjModel_set_tendon_solref_fri : mjModel -> float ptr -> unit

(** get tendon_solimp_fri from mjModel *)
val mjModel_get_tendon_solimp_fri : mjModel -> float ptr

(** set tendon_solimp_fri for mjModel *)
val mjModel_set_tendon_solimp_fri : mjModel -> float ptr -> unit

(** get tendon_range from mjModel *)
val mjModel_get_tendon_range : mjModel -> float ptr

(** set tendon_range for mjModel *)
val mjModel_set_tendon_range : mjModel -> float ptr -> unit

(** get tendon_margin from mjModel *)
val mjModel_get_tendon_margin : mjModel -> float ptr

(** set tendon_margin for mjModel *)
val mjModel_set_tendon_margin : mjModel -> float ptr -> unit

(** get tendon_stiffness from mjModel *)
val mjModel_get_tendon_stiffness : mjModel -> float ptr

(** set tendon_stiffness for mjModel *)
val mjModel_set_tendon_stiffness : mjModel -> float ptr -> unit

(** get tendon_damping from mjModel *)
val mjModel_get_tendon_damping : mjModel -> float ptr

(** set tendon_damping for mjModel *)
val mjModel_set_tendon_damping : mjModel -> float ptr -> unit

(** get tendon_frictionloss from mjModel *)
val mjModel_get_tendon_frictionloss : mjModel -> float ptr

(** set tendon_frictionloss for mjModel *)
val mjModel_set_tendon_frictionloss : mjModel -> float ptr -> unit

(** get tendon_lengthspring from mjModel *)
val mjModel_get_tendon_lengthspring : mjModel -> float ptr

(** set tendon_lengthspring for mjModel *)
val mjModel_set_tendon_lengthspring : mjModel -> float ptr -> unit

(** get tendon_length0 from mjModel *)
val mjModel_get_tendon_length0 : mjModel -> float ptr

(** set tendon_length0 for mjModel *)
val mjModel_set_tendon_length0 : mjModel -> float ptr -> unit

(** get tendon_invweight0 from mjModel *)
val mjModel_get_tendon_invweight0 : mjModel -> float ptr

(** set tendon_invweight0 for mjModel *)
val mjModel_set_tendon_invweight0 : mjModel -> float ptr -> unit

(** get tendon_user from mjModel *)
val mjModel_get_tendon_user : mjModel -> float ptr

(** set tendon_user for mjModel *)
val mjModel_set_tendon_user : mjModel -> float ptr -> unit

(** get tendon_rgba from mjModel *)
val mjModel_get_tendon_rgba : mjModel -> float ptr

(** set tendon_rgba for mjModel *)
val mjModel_set_tendon_rgba : mjModel -> float ptr -> unit

(** get wrap_type from mjModel *)
val mjModel_get_wrap_type : mjModel -> int ptr

(** set wrap_type for mjModel *)
val mjModel_set_wrap_type : mjModel -> int ptr -> unit

(** get wrap_objid from mjModel *)
val mjModel_get_wrap_objid : mjModel -> int ptr

(** set wrap_objid for mjModel *)
val mjModel_set_wrap_objid : mjModel -> int ptr -> unit

(** get wrap_prm from mjModel *)
val mjModel_get_wrap_prm : mjModel -> float ptr

(** set wrap_prm for mjModel *)
val mjModel_set_wrap_prm : mjModel -> float ptr -> unit

(** get actuator_trntype from mjModel *)
val mjModel_get_actuator_trntype : mjModel -> int ptr

(** set actuator_trntype for mjModel *)
val mjModel_set_actuator_trntype : mjModel -> int ptr -> unit

(** get actuator_dyntype from mjModel *)
val mjModel_get_actuator_dyntype : mjModel -> int ptr

(** set actuator_dyntype for mjModel *)
val mjModel_set_actuator_dyntype : mjModel -> int ptr -> unit

(** get actuator_gaintype from mjModel *)
val mjModel_get_actuator_gaintype : mjModel -> int ptr

(** set actuator_gaintype for mjModel *)
val mjModel_set_actuator_gaintype : mjModel -> int ptr -> unit

(** get actuator_biastype from mjModel *)
val mjModel_get_actuator_biastype : mjModel -> int ptr

(** set actuator_biastype for mjModel *)
val mjModel_set_actuator_biastype : mjModel -> int ptr -> unit

(** get actuator_trnid from mjModel *)
val mjModel_get_actuator_trnid : mjModel -> int ptr

(** set actuator_trnid for mjModel *)
val mjModel_set_actuator_trnid : mjModel -> int ptr -> unit

(** get actuator_group from mjModel *)
val mjModel_get_actuator_group : mjModel -> int ptr

(** set actuator_group for mjModel *)
val mjModel_set_actuator_group : mjModel -> int ptr -> unit

(** get actuator_ctrllimited from mjModel *)
val mjModel_get_actuator_ctrllimited : mjModel -> Unsigned.UChar.t ptr

(** set actuator_ctrllimited for mjModel *)
val mjModel_set_actuator_ctrllimited : mjModel -> Unsigned.UChar.t ptr -> unit

(** get actuator_forcelimited from mjModel *)
val mjModel_get_actuator_forcelimited : mjModel -> Unsigned.UChar.t ptr

(** set actuator_forcelimited for mjModel *)
val mjModel_set_actuator_forcelimited : mjModel -> Unsigned.UChar.t ptr -> unit

(** get actuator_dynprm from mjModel *)
val mjModel_get_actuator_dynprm : mjModel -> float ptr

(** set actuator_dynprm for mjModel *)
val mjModel_set_actuator_dynprm : mjModel -> float ptr -> unit

(** get actuator_gainprm from mjModel *)
val mjModel_get_actuator_gainprm : mjModel -> float ptr

(** set actuator_gainprm for mjModel *)
val mjModel_set_actuator_gainprm : mjModel -> float ptr -> unit

(** get actuator_biasprm from mjModel *)
val mjModel_get_actuator_biasprm : mjModel -> float ptr

(** set actuator_biasprm for mjModel *)
val mjModel_set_actuator_biasprm : mjModel -> float ptr -> unit

(** get actuator_ctrlrange from mjModel *)
val mjModel_get_actuator_ctrlrange : mjModel -> float ptr

(** set actuator_ctrlrange for mjModel *)
val mjModel_set_actuator_ctrlrange : mjModel -> float ptr -> unit

(** get actuator_forcerange from mjModel *)
val mjModel_get_actuator_forcerange : mjModel -> float ptr

(** set actuator_forcerange for mjModel *)
val mjModel_set_actuator_forcerange : mjModel -> float ptr -> unit

(** get actuator_gear from mjModel *)
val mjModel_get_actuator_gear : mjModel -> float ptr

(** set actuator_gear for mjModel *)
val mjModel_set_actuator_gear : mjModel -> float ptr -> unit

(** get actuator_cranklength from mjModel *)
val mjModel_get_actuator_cranklength : mjModel -> float ptr

(** set actuator_cranklength for mjModel *)
val mjModel_set_actuator_cranklength : mjModel -> float ptr -> unit

(** get actuator_acc0 from mjModel *)
val mjModel_get_actuator_acc0 : mjModel -> float ptr

(** set actuator_acc0 for mjModel *)
val mjModel_set_actuator_acc0 : mjModel -> float ptr -> unit

(** get actuator_length0 from mjModel *)
val mjModel_get_actuator_length0 : mjModel -> float ptr

(** set actuator_length0 for mjModel *)
val mjModel_set_actuator_length0 : mjModel -> float ptr -> unit

(** get actuator_lengthrange from mjModel *)
val mjModel_get_actuator_lengthrange : mjModel -> float ptr

(** set actuator_lengthrange for mjModel *)
val mjModel_set_actuator_lengthrange : mjModel -> float ptr -> unit

(** get actuator_user from mjModel *)
val mjModel_get_actuator_user : mjModel -> float ptr

(** set actuator_user for mjModel *)
val mjModel_set_actuator_user : mjModel -> float ptr -> unit

(** get sensor_type from mjModel *)
val mjModel_get_sensor_type : mjModel -> int ptr

(** set sensor_type for mjModel *)
val mjModel_set_sensor_type : mjModel -> int ptr -> unit

(** get sensor_datatype from mjModel *)
val mjModel_get_sensor_datatype : mjModel -> int ptr

(** set sensor_datatype for mjModel *)
val mjModel_set_sensor_datatype : mjModel -> int ptr -> unit

(** get sensor_needstage from mjModel *)
val mjModel_get_sensor_needstage : mjModel -> int ptr

(** set sensor_needstage for mjModel *)
val mjModel_set_sensor_needstage : mjModel -> int ptr -> unit

(** get sensor_objtype from mjModel *)
val mjModel_get_sensor_objtype : mjModel -> int ptr

(** set sensor_objtype for mjModel *)
val mjModel_set_sensor_objtype : mjModel -> int ptr -> unit

(** get sensor_objid from mjModel *)
val mjModel_get_sensor_objid : mjModel -> int ptr

(** set sensor_objid for mjModel *)
val mjModel_set_sensor_objid : mjModel -> int ptr -> unit

(** get sensor_dim from mjModel *)
val mjModel_get_sensor_dim : mjModel -> int ptr

(** set sensor_dim for mjModel *)
val mjModel_set_sensor_dim : mjModel -> int ptr -> unit

(** get sensor_adr from mjModel *)
val mjModel_get_sensor_adr : mjModel -> int ptr

(** set sensor_adr for mjModel *)
val mjModel_set_sensor_adr : mjModel -> int ptr -> unit

(** get sensor_cutoff from mjModel *)
val mjModel_get_sensor_cutoff : mjModel -> float ptr

(** set sensor_cutoff for mjModel *)
val mjModel_set_sensor_cutoff : mjModel -> float ptr -> unit

(** get sensor_noise from mjModel *)
val mjModel_get_sensor_noise : mjModel -> float ptr

(** set sensor_noise for mjModel *)
val mjModel_set_sensor_noise : mjModel -> float ptr -> unit

(** get sensor_user from mjModel *)
val mjModel_get_sensor_user : mjModel -> float ptr

(** set sensor_user for mjModel *)
val mjModel_set_sensor_user : mjModel -> float ptr -> unit

(** get numeric_adr from mjModel *)
val mjModel_get_numeric_adr : mjModel -> int ptr

(** set numeric_adr for mjModel *)
val mjModel_set_numeric_adr : mjModel -> int ptr -> unit

(** get numeric_size from mjModel *)
val mjModel_get_numeric_size : mjModel -> int ptr

(** set numeric_size for mjModel *)
val mjModel_set_numeric_size : mjModel -> int ptr -> unit

(** get numeric_data from mjModel *)
val mjModel_get_numeric_data : mjModel -> float ptr

(** set numeric_data for mjModel *)
val mjModel_set_numeric_data : mjModel -> float ptr -> unit

(** get text_adr from mjModel *)
val mjModel_get_text_adr : mjModel -> int ptr

(** set text_adr for mjModel *)
val mjModel_set_text_adr : mjModel -> int ptr -> unit

(** get text_size from mjModel *)
val mjModel_get_text_size : mjModel -> int ptr

(** set text_size for mjModel *)
val mjModel_set_text_size : mjModel -> int ptr -> unit

(** get text_data from mjModel *)
val mjModel_get_text_data : mjModel -> string

(** set text_data for mjModel *)
val mjModel_set_text_data : mjModel -> string -> unit

(** get tuple_adr from mjModel *)
val mjModel_get_tuple_adr : mjModel -> int ptr

(** set tuple_adr for mjModel *)
val mjModel_set_tuple_adr : mjModel -> int ptr -> unit

(** get tuple_size from mjModel *)
val mjModel_get_tuple_size : mjModel -> int ptr

(** set tuple_size for mjModel *)
val mjModel_set_tuple_size : mjModel -> int ptr -> unit

(** get tuple_objtype from mjModel *)
val mjModel_get_tuple_objtype : mjModel -> int ptr

(** set tuple_objtype for mjModel *)
val mjModel_set_tuple_objtype : mjModel -> int ptr -> unit

(** get tuple_objid from mjModel *)
val mjModel_get_tuple_objid : mjModel -> int ptr

(** set tuple_objid for mjModel *)
val mjModel_set_tuple_objid : mjModel -> int ptr -> unit

(** get tuple_objprm from mjModel *)
val mjModel_get_tuple_objprm : mjModel -> float ptr

(** set tuple_objprm for mjModel *)
val mjModel_set_tuple_objprm : mjModel -> float ptr -> unit

(** get key_time from mjModel *)
val mjModel_get_key_time : mjModel -> float ptr

(** set key_time for mjModel *)
val mjModel_set_key_time : mjModel -> float ptr -> unit

(** get key_qpos from mjModel *)
val mjModel_get_key_qpos : mjModel -> float ptr

(** set key_qpos for mjModel *)
val mjModel_set_key_qpos : mjModel -> float ptr -> unit

(** get key_qvel from mjModel *)
val mjModel_get_key_qvel : mjModel -> float ptr

(** set key_qvel for mjModel *)
val mjModel_set_key_qvel : mjModel -> float ptr -> unit

(** get key_act from mjModel *)
val mjModel_get_key_act : mjModel -> float ptr

(** set key_act for mjModel *)
val mjModel_set_key_act : mjModel -> float ptr -> unit

(** get key_mpos from mjModel *)
val mjModel_get_key_mpos : mjModel -> float ptr

(** set key_mpos for mjModel *)
val mjModel_set_key_mpos : mjModel -> float ptr -> unit

(** get key_mquat from mjModel *)
val mjModel_get_key_mquat : mjModel -> float ptr

(** set key_mquat for mjModel *)
val mjModel_set_key_mquat : mjModel -> float ptr -> unit

(** get name_bodyadr from mjModel *)
val mjModel_get_name_bodyadr : mjModel -> int ptr

(** set name_bodyadr for mjModel *)
val mjModel_set_name_bodyadr : mjModel -> int ptr -> unit

(** get name_jntadr from mjModel *)
val mjModel_get_name_jntadr : mjModel -> int ptr

(** set name_jntadr for mjModel *)
val mjModel_set_name_jntadr : mjModel -> int ptr -> unit

(** get name_geomadr from mjModel *)
val mjModel_get_name_geomadr : mjModel -> int ptr

(** set name_geomadr for mjModel *)
val mjModel_set_name_geomadr : mjModel -> int ptr -> unit

(** get name_siteadr from mjModel *)
val mjModel_get_name_siteadr : mjModel -> int ptr

(** set name_siteadr for mjModel *)
val mjModel_set_name_siteadr : mjModel -> int ptr -> unit

(** get name_camadr from mjModel *)
val mjModel_get_name_camadr : mjModel -> int ptr

(** set name_camadr for mjModel *)
val mjModel_set_name_camadr : mjModel -> int ptr -> unit

(** get name_lightadr from mjModel *)
val mjModel_get_name_lightadr : mjModel -> int ptr

(** set name_lightadr for mjModel *)
val mjModel_set_name_lightadr : mjModel -> int ptr -> unit

(** get name_meshadr from mjModel *)
val mjModel_get_name_meshadr : mjModel -> int ptr

(** set name_meshadr for mjModel *)
val mjModel_set_name_meshadr : mjModel -> int ptr -> unit

(** get name_skinadr from mjModel *)
val mjModel_get_name_skinadr : mjModel -> int ptr

(** set name_skinadr for mjModel *)
val mjModel_set_name_skinadr : mjModel -> int ptr -> unit

(** get name_hfieldadr from mjModel *)
val mjModel_get_name_hfieldadr : mjModel -> int ptr

(** set name_hfieldadr for mjModel *)
val mjModel_set_name_hfieldadr : mjModel -> int ptr -> unit

(** get name_texadr from mjModel *)
val mjModel_get_name_texadr : mjModel -> int ptr

(** set name_texadr for mjModel *)
val mjModel_set_name_texadr : mjModel -> int ptr -> unit

(** get name_matadr from mjModel *)
val mjModel_get_name_matadr : mjModel -> int ptr

(** set name_matadr for mjModel *)
val mjModel_set_name_matadr : mjModel -> int ptr -> unit

(** get name_pairadr from mjModel *)
val mjModel_get_name_pairadr : mjModel -> int ptr

(** set name_pairadr for mjModel *)
val mjModel_set_name_pairadr : mjModel -> int ptr -> unit

(** get name_excludeadr from mjModel *)
val mjModel_get_name_excludeadr : mjModel -> int ptr

(** set name_excludeadr for mjModel *)
val mjModel_set_name_excludeadr : mjModel -> int ptr -> unit

(** get name_eqadr from mjModel *)
val mjModel_get_name_eqadr : mjModel -> int ptr

(** set name_eqadr for mjModel *)
val mjModel_set_name_eqadr : mjModel -> int ptr -> unit

(** get name_tendonadr from mjModel *)
val mjModel_get_name_tendonadr : mjModel -> int ptr

(** set name_tendonadr for mjModel *)
val mjModel_set_name_tendonadr : mjModel -> int ptr -> unit

(** get name_actuatoradr from mjModel *)
val mjModel_get_name_actuatoradr : mjModel -> int ptr

(** set name_actuatoradr for mjModel *)
val mjModel_set_name_actuatoradr : mjModel -> int ptr -> unit

(** get name_sensoradr from mjModel *)
val mjModel_get_name_sensoradr : mjModel -> int ptr

(** set name_sensoradr for mjModel *)
val mjModel_set_name_sensoradr : mjModel -> int ptr -> unit

(** get name_numericadr from mjModel *)
val mjModel_get_name_numericadr : mjModel -> int ptr

(** set name_numericadr for mjModel *)
val mjModel_set_name_numericadr : mjModel -> int ptr -> unit

(** get name_textadr from mjModel *)
val mjModel_get_name_textadr : mjModel -> int ptr

(** set name_textadr for mjModel *)
val mjModel_set_name_textadr : mjModel -> int ptr -> unit

(** get name_tupleadr from mjModel *)
val mjModel_get_name_tupleadr : mjModel -> int ptr

(** set name_tupleadr for mjModel *)
val mjModel_set_name_tupleadr : mjModel -> int ptr -> unit

(** get name_keyadr from mjModel *)
val mjModel_get_name_keyadr : mjModel -> int ptr

(** set name_keyadr for mjModel *)
val mjModel_set_name_keyadr : mjModel -> int ptr -> unit

(** get names from mjModel *)
val mjModel_get_names : mjModel -> string

(** set names for mjModel *)
val mjModel_set_names : mjModel -> string -> unit

(** convert mjtWarning type to int *)
val mjtWarning_to_int : mjtWarning -> int

(** convert mjtTimer type to int *)
val mjtTimer_to_int : mjtTimer -> int

(** allocate fresh mjContact struct *)
val mjContact_allocate : unit -> mjContact

(** make null mjContact struct ptr *)
val mjContact_null : unit -> mjContact ptr

(** get dist from mjContact *)
val mjContact_get_dist : mjContact -> float

(** set dist for mjContact *)
val mjContact_set_dist : mjContact -> float -> unit

(** get pos from mjContact *)
val mjContact_get_pos : mjContact -> float ptr

(** set pos for mjContact *)
val mjContact_set_pos : mjContact -> float ptr -> unit

(** get frame from mjContact *)
val mjContact_get_frame : mjContact -> float ptr

(** set frame for mjContact *)
val mjContact_set_frame : mjContact -> float ptr -> unit

(** get includemargin from mjContact *)
val mjContact_get_includemargin : mjContact -> float

(** set includemargin for mjContact *)
val mjContact_set_includemargin : mjContact -> float -> unit

(** get friction from mjContact *)
val mjContact_get_friction : mjContact -> float ptr

(** set friction for mjContact *)
val mjContact_set_friction : mjContact -> float ptr -> unit

(** get solref from mjContact *)
val mjContact_get_solref : mjContact -> float ptr

(** set solref for mjContact *)
val mjContact_set_solref : mjContact -> float ptr -> unit

(** get solimp from mjContact *)
val mjContact_get_solimp : mjContact -> float ptr

(** set solimp for mjContact *)
val mjContact_set_solimp : mjContact -> float ptr -> unit

(** get mu from mjContact *)
val mjContact_get_mu : mjContact -> float

(** set mu for mjContact *)
val mjContact_set_mu : mjContact -> float -> unit

(** get H from mjContact *)
val mjContact_get_H : mjContact -> float ptr

(** set H for mjContact *)
val mjContact_set_H : mjContact -> float ptr -> unit

(** get dim from mjContact *)
val mjContact_get_dim : mjContact -> int

(** set dim for mjContact *)
val mjContact_set_dim : mjContact -> int -> unit

(** get geom1 from mjContact *)
val mjContact_get_geom1 : mjContact -> int

(** set geom1 for mjContact *)
val mjContact_set_geom1 : mjContact -> int -> unit

(** get geom2 from mjContact *)
val mjContact_get_geom2 : mjContact -> int

(** set geom2 for mjContact *)
val mjContact_set_geom2 : mjContact -> int -> unit

(** get exclude from mjContact *)
val mjContact_get_exclude : mjContact -> int

(** set exclude for mjContact *)
val mjContact_set_exclude : mjContact -> int -> unit

(** get efc_address from mjContact *)
val mjContact_get_efc_address : mjContact -> int

(** set efc_address for mjContact *)
val mjContact_set_efc_address : mjContact -> int -> unit

(** allocate fresh mjWarningStat struct *)
val mjWarningStat_allocate : unit -> mjWarningStat

(** make null mjWarningStat struct ptr *)
val mjWarningStat_null : unit -> mjWarningStat ptr

(** get lastinfo from mjWarningStat *)
val mjWarningStat_get_lastinfo : mjWarningStat -> int

(** set lastinfo for mjWarningStat *)
val mjWarningStat_set_lastinfo : mjWarningStat -> int -> unit

(** get number from mjWarningStat *)
val mjWarningStat_get_number : mjWarningStat -> int

(** set number for mjWarningStat *)
val mjWarningStat_set_number : mjWarningStat -> int -> unit

(** allocate fresh mjTimerStat struct *)
val mjTimerStat_allocate : unit -> mjTimerStat

(** make null mjTimerStat struct ptr *)
val mjTimerStat_null : unit -> mjTimerStat ptr

(** get duration from mjTimerStat *)
val mjTimerStat_get_duration : mjTimerStat -> float

(** set duration for mjTimerStat *)
val mjTimerStat_set_duration : mjTimerStat -> float -> unit

(** get number from mjTimerStat *)
val mjTimerStat_get_number : mjTimerStat -> int

(** set number for mjTimerStat *)
val mjTimerStat_set_number : mjTimerStat -> int -> unit

(** allocate fresh mjSolverStat struct *)
val mjSolverStat_allocate : unit -> mjSolverStat

(** make null mjSolverStat struct ptr *)
val mjSolverStat_null : unit -> mjSolverStat ptr

(** get improvement from mjSolverStat *)
val mjSolverStat_get_improvement : mjSolverStat -> float

(** set improvement for mjSolverStat *)
val mjSolverStat_set_improvement : mjSolverStat -> float -> unit

(** get gradient from mjSolverStat *)
val mjSolverStat_get_gradient : mjSolverStat -> float

(** set gradient for mjSolverStat *)
val mjSolverStat_set_gradient : mjSolverStat -> float -> unit

(** get lineslope from mjSolverStat *)
val mjSolverStat_get_lineslope : mjSolverStat -> float

(** set lineslope for mjSolverStat *)
val mjSolverStat_set_lineslope : mjSolverStat -> float -> unit

(** get nactive from mjSolverStat *)
val mjSolverStat_get_nactive : mjSolverStat -> int

(** set nactive for mjSolverStat *)
val mjSolverStat_set_nactive : mjSolverStat -> int -> unit

(** get nchange from mjSolverStat *)
val mjSolverStat_get_nchange : mjSolverStat -> int

(** set nchange for mjSolverStat *)
val mjSolverStat_set_nchange : mjSolverStat -> int -> unit

(** get neval from mjSolverStat *)
val mjSolverStat_get_neval : mjSolverStat -> int

(** set neval for mjSolverStat *)
val mjSolverStat_set_neval : mjSolverStat -> int -> unit

(** get nupdate from mjSolverStat *)
val mjSolverStat_get_nupdate : mjSolverStat -> int

(** set nupdate for mjSolverStat *)
val mjSolverStat_set_nupdate : mjSolverStat -> int -> unit

(** allocate fresh mjData struct *)
val mjData_allocate : unit -> mjData

(** make null mjData struct ptr *)
val mjData_null : unit -> mjData ptr

(** get nstack from mjData *)
val mjData_get_nstack : mjData -> int

(** set nstack for mjData *)
val mjData_set_nstack : mjData -> int -> unit

(** get nbuffer from mjData *)
val mjData_get_nbuffer : mjData -> int

(** set nbuffer for mjData *)
val mjData_set_nbuffer : mjData -> int -> unit

(** get pstack from mjData *)
val mjData_get_pstack : mjData -> int

(** set pstack for mjData *)
val mjData_set_pstack : mjData -> int -> unit

(** get maxuse_stack from mjData *)
val mjData_get_maxuse_stack : mjData -> int

(** set maxuse_stack for mjData *)
val mjData_set_maxuse_stack : mjData -> int -> unit

(** get maxuse_con from mjData *)
val mjData_get_maxuse_con : mjData -> int

(** set maxuse_con for mjData *)
val mjData_set_maxuse_con : mjData -> int -> unit

(** get maxuse_efc from mjData *)
val mjData_get_maxuse_efc : mjData -> int

(** set maxuse_efc for mjData *)
val mjData_set_maxuse_efc : mjData -> int -> unit

(** get warning from mjData *)
val mjData_get_warning : mjData -> mjWarningStat ptr

(** set warning for mjData *)
val mjData_set_warning : mjData -> mjWarningStat ptr -> unit

(** get timer from mjData *)
val mjData_get_timer : mjData -> mjTimerStat ptr

(** set timer for mjData *)
val mjData_set_timer : mjData -> mjTimerStat ptr -> unit

(** get solver from mjData *)
val mjData_get_solver : mjData -> mjSolverStat ptr

(** set solver for mjData *)
val mjData_set_solver : mjData -> mjSolverStat ptr -> unit

(** get solver_iter from mjData *)
val mjData_get_solver_iter : mjData -> int

(** set solver_iter for mjData *)
val mjData_set_solver_iter : mjData -> int -> unit

(** get solver_nnz from mjData *)
val mjData_get_solver_nnz : mjData -> int

(** set solver_nnz for mjData *)
val mjData_set_solver_nnz : mjData -> int -> unit

(** get solver_fwdinv from mjData *)
val mjData_get_solver_fwdinv : mjData -> float ptr

(** set solver_fwdinv for mjData *)
val mjData_set_solver_fwdinv : mjData -> float ptr -> unit

(** get ne from mjData *)
val mjData_get_ne : mjData -> int

(** set ne for mjData *)
val mjData_set_ne : mjData -> int -> unit

(** get nf from mjData *)
val mjData_get_nf : mjData -> int

(** set nf for mjData *)
val mjData_set_nf : mjData -> int -> unit

(** get nefc from mjData *)
val mjData_get_nefc : mjData -> int

(** set nefc for mjData *)
val mjData_set_nefc : mjData -> int -> unit

(** get ncon from mjData *)
val mjData_get_ncon : mjData -> int

(** set ncon for mjData *)
val mjData_set_ncon : mjData -> int -> unit

(** get time from mjData *)
val mjData_get_time : mjData -> float

(** set time for mjData *)
val mjData_set_time : mjData -> float -> unit

(** get energy from mjData *)
val mjData_get_energy : mjData -> float ptr

(** set energy for mjData *)
val mjData_set_energy : mjData -> float ptr -> unit

(** get buffer from mjData *)
val mjData_get_buffer : mjData -> unit ptr

(** set buffer for mjData *)
val mjData_set_buffer : mjData -> unit ptr -> unit

(** get stack from mjData *)
val mjData_get_stack : mjData -> float ptr

(** set stack for mjData *)
val mjData_set_stack : mjData -> float ptr -> unit

(** get qpos from mjData *)
val mjData_get_qpos : mjData -> float ptr

(** set qpos for mjData *)
val mjData_set_qpos : mjData -> float ptr -> unit

(** get qvel from mjData *)
val mjData_get_qvel : mjData -> float ptr

(** set qvel for mjData *)
val mjData_set_qvel : mjData -> float ptr -> unit

(** get act from mjData *)
val mjData_get_act : mjData -> float ptr

(** set act for mjData *)
val mjData_set_act : mjData -> float ptr -> unit

(** get qacc_warmstart from mjData *)
val mjData_get_qacc_warmstart : mjData -> float ptr

(** set qacc_warmstart for mjData *)
val mjData_set_qacc_warmstart : mjData -> float ptr -> unit

(** get ctrl from mjData *)
val mjData_get_ctrl : mjData -> float ptr

(** set ctrl for mjData *)
val mjData_set_ctrl : mjData -> float ptr -> unit

(** get qfrc_applied from mjData *)
val mjData_get_qfrc_applied : mjData -> float ptr

(** set qfrc_applied for mjData *)
val mjData_set_qfrc_applied : mjData -> float ptr -> unit

(** get xfrc_applied from mjData *)
val mjData_get_xfrc_applied : mjData -> float ptr

(** set xfrc_applied for mjData *)
val mjData_set_xfrc_applied : mjData -> float ptr -> unit

(** get qacc from mjData *)
val mjData_get_qacc : mjData -> float ptr

(** set qacc for mjData *)
val mjData_set_qacc : mjData -> float ptr -> unit

(** get act_dot from mjData *)
val mjData_get_act_dot : mjData -> float ptr

(** set act_dot for mjData *)
val mjData_set_act_dot : mjData -> float ptr -> unit

(** get mocap_pos from mjData *)
val mjData_get_mocap_pos : mjData -> float ptr

(** set mocap_pos for mjData *)
val mjData_set_mocap_pos : mjData -> float ptr -> unit

(** get mocap_quat from mjData *)
val mjData_get_mocap_quat : mjData -> float ptr

(** set mocap_quat for mjData *)
val mjData_set_mocap_quat : mjData -> float ptr -> unit

(** get userdata from mjData *)
val mjData_get_userdata : mjData -> float ptr

(** set userdata for mjData *)
val mjData_set_userdata : mjData -> float ptr -> unit

(** get sensordata from mjData *)
val mjData_get_sensordata : mjData -> float ptr

(** set sensordata for mjData *)
val mjData_set_sensordata : mjData -> float ptr -> unit

(** get xpos from mjData *)
val mjData_get_xpos : mjData -> float ptr

(** set xpos for mjData *)
val mjData_set_xpos : mjData -> float ptr -> unit

(** get xquat from mjData *)
val mjData_get_xquat : mjData -> float ptr

(** set xquat for mjData *)
val mjData_set_xquat : mjData -> float ptr -> unit

(** get xmat from mjData *)
val mjData_get_xmat : mjData -> float ptr

(** set xmat for mjData *)
val mjData_set_xmat : mjData -> float ptr -> unit

(** get xipos from mjData *)
val mjData_get_xipos : mjData -> float ptr

(** set xipos for mjData *)
val mjData_set_xipos : mjData -> float ptr -> unit

(** get ximat from mjData *)
val mjData_get_ximat : mjData -> float ptr

(** set ximat for mjData *)
val mjData_set_ximat : mjData -> float ptr -> unit

(** get xanchor from mjData *)
val mjData_get_xanchor : mjData -> float ptr

(** set xanchor for mjData *)
val mjData_set_xanchor : mjData -> float ptr -> unit

(** get xaxis from mjData *)
val mjData_get_xaxis : mjData -> float ptr

(** set xaxis for mjData *)
val mjData_set_xaxis : mjData -> float ptr -> unit

(** get geom_xpos from mjData *)
val mjData_get_geom_xpos : mjData -> float ptr

(** set geom_xpos for mjData *)
val mjData_set_geom_xpos : mjData -> float ptr -> unit

(** get geom_xmat from mjData *)
val mjData_get_geom_xmat : mjData -> float ptr

(** set geom_xmat for mjData *)
val mjData_set_geom_xmat : mjData -> float ptr -> unit

(** get site_xpos from mjData *)
val mjData_get_site_xpos : mjData -> float ptr

(** set site_xpos for mjData *)
val mjData_set_site_xpos : mjData -> float ptr -> unit

(** get site_xmat from mjData *)
val mjData_get_site_xmat : mjData -> float ptr

(** set site_xmat for mjData *)
val mjData_set_site_xmat : mjData -> float ptr -> unit

(** get cam_xpos from mjData *)
val mjData_get_cam_xpos : mjData -> float ptr

(** set cam_xpos for mjData *)
val mjData_set_cam_xpos : mjData -> float ptr -> unit

(** get cam_xmat from mjData *)
val mjData_get_cam_xmat : mjData -> float ptr

(** set cam_xmat for mjData *)
val mjData_set_cam_xmat : mjData -> float ptr -> unit

(** get light_xpos from mjData *)
val mjData_get_light_xpos : mjData -> float ptr

(** set light_xpos for mjData *)
val mjData_set_light_xpos : mjData -> float ptr -> unit

(** get light_xdir from mjData *)
val mjData_get_light_xdir : mjData -> float ptr

(** set light_xdir for mjData *)
val mjData_set_light_xdir : mjData -> float ptr -> unit

(** get subtree_com from mjData *)
val mjData_get_subtree_com : mjData -> float ptr

(** set subtree_com for mjData *)
val mjData_set_subtree_com : mjData -> float ptr -> unit

(** get cdof from mjData *)
val mjData_get_cdof : mjData -> float ptr

(** set cdof for mjData *)
val mjData_set_cdof : mjData -> float ptr -> unit

(** get cinert from mjData *)
val mjData_get_cinert : mjData -> float ptr

(** set cinert for mjData *)
val mjData_set_cinert : mjData -> float ptr -> unit

(** get ten_wrapadr from mjData *)
val mjData_get_ten_wrapadr : mjData -> int ptr

(** set ten_wrapadr for mjData *)
val mjData_set_ten_wrapadr : mjData -> int ptr -> unit

(** get ten_wrapnum from mjData *)
val mjData_get_ten_wrapnum : mjData -> int ptr

(** set ten_wrapnum for mjData *)
val mjData_set_ten_wrapnum : mjData -> int ptr -> unit

(** get ten_J_rownnz from mjData *)
val mjData_get_ten_J_rownnz : mjData -> int ptr

(** set ten_J_rownnz for mjData *)
val mjData_set_ten_J_rownnz : mjData -> int ptr -> unit

(** get ten_J_rowadr from mjData *)
val mjData_get_ten_J_rowadr : mjData -> int ptr

(** set ten_J_rowadr for mjData *)
val mjData_set_ten_J_rowadr : mjData -> int ptr -> unit

(** get ten_J_colind from mjData *)
val mjData_get_ten_J_colind : mjData -> int ptr

(** set ten_J_colind for mjData *)
val mjData_set_ten_J_colind : mjData -> int ptr -> unit

(** get ten_length from mjData *)
val mjData_get_ten_length : mjData -> float ptr

(** set ten_length for mjData *)
val mjData_set_ten_length : mjData -> float ptr -> unit

(** get ten_J from mjData *)
val mjData_get_ten_J : mjData -> float ptr

(** set ten_J for mjData *)
val mjData_set_ten_J : mjData -> float ptr -> unit

(** get wrap_obj from mjData *)
val mjData_get_wrap_obj : mjData -> int ptr

(** set wrap_obj for mjData *)
val mjData_set_wrap_obj : mjData -> int ptr -> unit

(** get wrap_xpos from mjData *)
val mjData_get_wrap_xpos : mjData -> float ptr

(** set wrap_xpos for mjData *)
val mjData_set_wrap_xpos : mjData -> float ptr -> unit

(** get actuator_length from mjData *)
val mjData_get_actuator_length : mjData -> float ptr

(** set actuator_length for mjData *)
val mjData_set_actuator_length : mjData -> float ptr -> unit

(** get actuator_moment from mjData *)
val mjData_get_actuator_moment : mjData -> float ptr

(** set actuator_moment for mjData *)
val mjData_set_actuator_moment : mjData -> float ptr -> unit

(** get crb from mjData *)
val mjData_get_crb : mjData -> float ptr

(** set crb for mjData *)
val mjData_set_crb : mjData -> float ptr -> unit

(** get qM from mjData *)
val mjData_get_qM : mjData -> float ptr

(** set qM for mjData *)
val mjData_set_qM : mjData -> float ptr -> unit

(** get qLD from mjData *)
val mjData_get_qLD : mjData -> float ptr

(** set qLD for mjData *)
val mjData_set_qLD : mjData -> float ptr -> unit

(** get qLDiagInv from mjData *)
val mjData_get_qLDiagInv : mjData -> float ptr

(** set qLDiagInv for mjData *)
val mjData_set_qLDiagInv : mjData -> float ptr -> unit

(** get qLDiagSqrtInv from mjData *)
val mjData_get_qLDiagSqrtInv : mjData -> float ptr

(** set qLDiagSqrtInv for mjData *)
val mjData_set_qLDiagSqrtInv : mjData -> float ptr -> unit

(** get contact from mjData *)
val mjData_get_contact : mjData -> mjContact ptr

(** set contact for mjData *)
val mjData_set_contact : mjData -> mjContact ptr -> unit

(** get efc_type from mjData *)
val mjData_get_efc_type : mjData -> int ptr

(** set efc_type for mjData *)
val mjData_set_efc_type : mjData -> int ptr -> unit

(** get efc_id from mjData *)
val mjData_get_efc_id : mjData -> int ptr

(** set efc_id for mjData *)
val mjData_set_efc_id : mjData -> int ptr -> unit

(** get efc_J_rownnz from mjData *)
val mjData_get_efc_J_rownnz : mjData -> int ptr

(** set efc_J_rownnz for mjData *)
val mjData_set_efc_J_rownnz : mjData -> int ptr -> unit

(** get efc_J_rowadr from mjData *)
val mjData_get_efc_J_rowadr : mjData -> int ptr

(** set efc_J_rowadr for mjData *)
val mjData_set_efc_J_rowadr : mjData -> int ptr -> unit

(** get efc_J_rowsuper from mjData *)
val mjData_get_efc_J_rowsuper : mjData -> int ptr

(** set efc_J_rowsuper for mjData *)
val mjData_set_efc_J_rowsuper : mjData -> int ptr -> unit

(** get efc_J_colind from mjData *)
val mjData_get_efc_J_colind : mjData -> int ptr

(** set efc_J_colind for mjData *)
val mjData_set_efc_J_colind : mjData -> int ptr -> unit

(** get efc_JT_rownnz from mjData *)
val mjData_get_efc_JT_rownnz : mjData -> int ptr

(** set efc_JT_rownnz for mjData *)
val mjData_set_efc_JT_rownnz : mjData -> int ptr -> unit

(** get efc_JT_rowadr from mjData *)
val mjData_get_efc_JT_rowadr : mjData -> int ptr

(** set efc_JT_rowadr for mjData *)
val mjData_set_efc_JT_rowadr : mjData -> int ptr -> unit

(** get efc_JT_rowsuper from mjData *)
val mjData_get_efc_JT_rowsuper : mjData -> int ptr

(** set efc_JT_rowsuper for mjData *)
val mjData_set_efc_JT_rowsuper : mjData -> int ptr -> unit

(** get efc_JT_colind from mjData *)
val mjData_get_efc_JT_colind : mjData -> int ptr

(** set efc_JT_colind for mjData *)
val mjData_set_efc_JT_colind : mjData -> int ptr -> unit

(** get efc_J from mjData *)
val mjData_get_efc_J : mjData -> float ptr

(** set efc_J for mjData *)
val mjData_set_efc_J : mjData -> float ptr -> unit

(** get efc_JT from mjData *)
val mjData_get_efc_JT : mjData -> float ptr

(** set efc_JT for mjData *)
val mjData_set_efc_JT : mjData -> float ptr -> unit

(** get efc_pos from mjData *)
val mjData_get_efc_pos : mjData -> float ptr

(** set efc_pos for mjData *)
val mjData_set_efc_pos : mjData -> float ptr -> unit

(** get efc_margin from mjData *)
val mjData_get_efc_margin : mjData -> float ptr

(** set efc_margin for mjData *)
val mjData_set_efc_margin : mjData -> float ptr -> unit

(** get efc_frictionloss from mjData *)
val mjData_get_efc_frictionloss : mjData -> float ptr

(** set efc_frictionloss for mjData *)
val mjData_set_efc_frictionloss : mjData -> float ptr -> unit

(** get efc_diagApprox from mjData *)
val mjData_get_efc_diagApprox : mjData -> float ptr

(** set efc_diagApprox for mjData *)
val mjData_set_efc_diagApprox : mjData -> float ptr -> unit

(** get efc_KBIP from mjData *)
val mjData_get_efc_KBIP : mjData -> float ptr

(** set efc_KBIP for mjData *)
val mjData_set_efc_KBIP : mjData -> float ptr -> unit

(** get efc_D from mjData *)
val mjData_get_efc_D : mjData -> float ptr

(** set efc_D for mjData *)
val mjData_set_efc_D : mjData -> float ptr -> unit

(** get efc_R from mjData *)
val mjData_get_efc_R : mjData -> float ptr

(** set efc_R for mjData *)
val mjData_set_efc_R : mjData -> float ptr -> unit

(** get efc_AR_rownnz from mjData *)
val mjData_get_efc_AR_rownnz : mjData -> int ptr

(** set efc_AR_rownnz for mjData *)
val mjData_set_efc_AR_rownnz : mjData -> int ptr -> unit

(** get efc_AR_rowadr from mjData *)
val mjData_get_efc_AR_rowadr : mjData -> int ptr

(** set efc_AR_rowadr for mjData *)
val mjData_set_efc_AR_rowadr : mjData -> int ptr -> unit

(** get efc_AR_colind from mjData *)
val mjData_get_efc_AR_colind : mjData -> int ptr

(** set efc_AR_colind for mjData *)
val mjData_set_efc_AR_colind : mjData -> int ptr -> unit

(** get efc_AR from mjData *)
val mjData_get_efc_AR : mjData -> float ptr

(** set efc_AR for mjData *)
val mjData_set_efc_AR : mjData -> float ptr -> unit

(** get ten_velocity from mjData *)
val mjData_get_ten_velocity : mjData -> float ptr

(** set ten_velocity for mjData *)
val mjData_set_ten_velocity : mjData -> float ptr -> unit

(** get actuator_velocity from mjData *)
val mjData_get_actuator_velocity : mjData -> float ptr

(** set actuator_velocity for mjData *)
val mjData_set_actuator_velocity : mjData -> float ptr -> unit

(** get cvel from mjData *)
val mjData_get_cvel : mjData -> float ptr

(** set cvel for mjData *)
val mjData_set_cvel : mjData -> float ptr -> unit

(** get cdof_dot from mjData *)
val mjData_get_cdof_dot : mjData -> float ptr

(** set cdof_dot for mjData *)
val mjData_set_cdof_dot : mjData -> float ptr -> unit

(** get qfrc_bias from mjData *)
val mjData_get_qfrc_bias : mjData -> float ptr

(** set qfrc_bias for mjData *)
val mjData_set_qfrc_bias : mjData -> float ptr -> unit

(** get qfrc_passive from mjData *)
val mjData_get_qfrc_passive : mjData -> float ptr

(** set qfrc_passive for mjData *)
val mjData_set_qfrc_passive : mjData -> float ptr -> unit

(** get efc_vel from mjData *)
val mjData_get_efc_vel : mjData -> float ptr

(** set efc_vel for mjData *)
val mjData_set_efc_vel : mjData -> float ptr -> unit

(** get efc_aref from mjData *)
val mjData_get_efc_aref : mjData -> float ptr

(** set efc_aref for mjData *)
val mjData_set_efc_aref : mjData -> float ptr -> unit

(** get subtree_linvel from mjData *)
val mjData_get_subtree_linvel : mjData -> float ptr

(** set subtree_linvel for mjData *)
val mjData_set_subtree_linvel : mjData -> float ptr -> unit

(** get subtree_angmom from mjData *)
val mjData_get_subtree_angmom : mjData -> float ptr

(** set subtree_angmom for mjData *)
val mjData_set_subtree_angmom : mjData -> float ptr -> unit

(** get actuator_force from mjData *)
val mjData_get_actuator_force : mjData -> float ptr

(** set actuator_force for mjData *)
val mjData_set_actuator_force : mjData -> float ptr -> unit

(** get qfrc_actuator from mjData *)
val mjData_get_qfrc_actuator : mjData -> float ptr

(** set qfrc_actuator for mjData *)
val mjData_set_qfrc_actuator : mjData -> float ptr -> unit

(** get qfrc_unc from mjData *)
val mjData_get_qfrc_unc : mjData -> float ptr

(** set qfrc_unc for mjData *)
val mjData_set_qfrc_unc : mjData -> float ptr -> unit

(** get qacc_unc from mjData *)
val mjData_get_qacc_unc : mjData -> float ptr

(** set qacc_unc for mjData *)
val mjData_set_qacc_unc : mjData -> float ptr -> unit

(** get efc_b from mjData *)
val mjData_get_efc_b : mjData -> float ptr

(** set efc_b for mjData *)
val mjData_set_efc_b : mjData -> float ptr -> unit

(** get efc_force from mjData *)
val mjData_get_efc_force : mjData -> float ptr

(** set efc_force for mjData *)
val mjData_set_efc_force : mjData -> float ptr -> unit

(** get efc_state from mjData *)
val mjData_get_efc_state : mjData -> int ptr

(** set efc_state for mjData *)
val mjData_set_efc_state : mjData -> int ptr -> unit

(** get qfrc_constraint from mjData *)
val mjData_get_qfrc_constraint : mjData -> float ptr

(** set qfrc_constraint for mjData *)
val mjData_set_qfrc_constraint : mjData -> float ptr -> unit

(** get qfrc_inverse from mjData *)
val mjData_get_qfrc_inverse : mjData -> float ptr

(** set qfrc_inverse for mjData *)
val mjData_set_qfrc_inverse : mjData -> float ptr -> unit

(** get cacc from mjData *)
val mjData_get_cacc : mjData -> float ptr

(** set cacc for mjData *)
val mjData_set_cacc : mjData -> float ptr -> unit

(** get cfrc_int from mjData *)
val mjData_get_cfrc_int : mjData -> float ptr

(** set cfrc_int for mjData *)
val mjData_set_cfrc_int : mjData -> float ptr -> unit

(** get cfrc_ext from mjData *)
val mjData_get_cfrc_ext : mjData -> float ptr

(** set cfrc_ext for mjData *)
val mjData_set_cfrc_ext : mjData -> float ptr -> unit

(** convert mjtCatBit type to int *)
val mjtCatBit_to_int : mjtCatBit -> int

(** convert mjtMouse type to int *)
val mjtMouse_to_int : mjtMouse -> int

(** convert mjtPertBit type to int *)
val mjtPertBit_to_int : mjtPertBit -> int

(** convert mjtCamera type to int *)
val mjtCamera_to_int : mjtCamera -> int

(** convert mjtLabel type to int *)
val mjtLabel_to_int : mjtLabel -> int

(** convert mjtFrame type to int *)
val mjtFrame_to_int : mjtFrame -> int

(** convert mjtVisFlag type to int *)
val mjtVisFlag_to_int : mjtVisFlag -> int

(** convert mjtRndFlag type to int *)
val mjtRndFlag_to_int : mjtRndFlag -> int

(** convert mjtStereo type to int *)
val mjtStereo_to_int : mjtStereo -> int

(** allocate fresh mjvPerturb struct *)
val mjvPerturb_allocate : unit -> mjvPerturb

(** make null mjvPerturb struct ptr *)
val mjvPerturb_null : unit -> mjvPerturb ptr

(** get select from mjvPerturb *)
val mjvPerturb_get_select : mjvPerturb -> int

(** set select for mjvPerturb *)
val mjvPerturb_set_select : mjvPerturb -> int -> unit

(** get skinselect from mjvPerturb *)
val mjvPerturb_get_skinselect : mjvPerturb -> int

(** set skinselect for mjvPerturb *)
val mjvPerturb_set_skinselect : mjvPerturb -> int -> unit

(** get active from mjvPerturb *)
val mjvPerturb_get_active : mjvPerturb -> int

(** set active for mjvPerturb *)
val mjvPerturb_set_active : mjvPerturb -> int -> unit

(** get active2 from mjvPerturb *)
val mjvPerturb_get_active2 : mjvPerturb -> int

(** set active2 for mjvPerturb *)
val mjvPerturb_set_active2 : mjvPerturb -> int -> unit

(** get refpos from mjvPerturb *)
val mjvPerturb_get_refpos : mjvPerturb -> float ptr

(** set refpos for mjvPerturb *)
val mjvPerturb_set_refpos : mjvPerturb -> float ptr -> unit

(** get refquat from mjvPerturb *)
val mjvPerturb_get_refquat : mjvPerturb -> float ptr

(** set refquat for mjvPerturb *)
val mjvPerturb_set_refquat : mjvPerturb -> float ptr -> unit

(** get localpos from mjvPerturb *)
val mjvPerturb_get_localpos : mjvPerturb -> float ptr

(** set localpos for mjvPerturb *)
val mjvPerturb_set_localpos : mjvPerturb -> float ptr -> unit

(** get scale from mjvPerturb *)
val mjvPerturb_get_scale : mjvPerturb -> float

(** set scale for mjvPerturb *)
val mjvPerturb_set_scale : mjvPerturb -> float -> unit

(** allocate fresh mjvCamera struct *)
val mjvCamera_allocate : unit -> mjvCamera

(** make null mjvCamera struct ptr *)
val mjvCamera_null : unit -> mjvCamera ptr

(** get type from mjvCamera *)
val mjvCamera_get_type : mjvCamera -> int

(** set type for mjvCamera *)
val mjvCamera_set_type : mjvCamera -> int -> unit

(** get fixedcamid from mjvCamera *)
val mjvCamera_get_fixedcamid : mjvCamera -> int

(** set fixedcamid for mjvCamera *)
val mjvCamera_set_fixedcamid : mjvCamera -> int -> unit

(** get trackbodyid from mjvCamera *)
val mjvCamera_get_trackbodyid : mjvCamera -> int

(** set trackbodyid for mjvCamera *)
val mjvCamera_set_trackbodyid : mjvCamera -> int -> unit

(** get lookat from mjvCamera *)
val mjvCamera_get_lookat : mjvCamera -> float ptr

(** set lookat for mjvCamera *)
val mjvCamera_set_lookat : mjvCamera -> float ptr -> unit

(** get distance from mjvCamera *)
val mjvCamera_get_distance : mjvCamera -> float

(** set distance for mjvCamera *)
val mjvCamera_set_distance : mjvCamera -> float -> unit

(** get azimuth from mjvCamera *)
val mjvCamera_get_azimuth : mjvCamera -> float

(** set azimuth for mjvCamera *)
val mjvCamera_set_azimuth : mjvCamera -> float -> unit

(** get elevation from mjvCamera *)
val mjvCamera_get_elevation : mjvCamera -> float

(** set elevation for mjvCamera *)
val mjvCamera_set_elevation : mjvCamera -> float -> unit

(** allocate fresh mjvGLCamera struct *)
val mjvGLCamera_allocate : unit -> mjvGLCamera

(** make null mjvGLCamera struct ptr *)
val mjvGLCamera_null : unit -> mjvGLCamera ptr

(** get pos from mjvGLCamera *)
val mjvGLCamera_get_pos : mjvGLCamera -> float ptr

(** set pos for mjvGLCamera *)
val mjvGLCamera_set_pos : mjvGLCamera -> float ptr -> unit

(** get forward from mjvGLCamera *)
val mjvGLCamera_get_forward : mjvGLCamera -> float ptr

(** set forward for mjvGLCamera *)
val mjvGLCamera_set_forward : mjvGLCamera -> float ptr -> unit

(** get up from mjvGLCamera *)
val mjvGLCamera_get_up : mjvGLCamera -> float ptr

(** set up for mjvGLCamera *)
val mjvGLCamera_set_up : mjvGLCamera -> float ptr -> unit

(** get frustum_center from mjvGLCamera *)
val mjvGLCamera_get_frustum_center : mjvGLCamera -> float

(** set frustum_center for mjvGLCamera *)
val mjvGLCamera_set_frustum_center : mjvGLCamera -> float -> unit

(** get frustum_bottom from mjvGLCamera *)
val mjvGLCamera_get_frustum_bottom : mjvGLCamera -> float

(** set frustum_bottom for mjvGLCamera *)
val mjvGLCamera_set_frustum_bottom : mjvGLCamera -> float -> unit

(** get frustum_top from mjvGLCamera *)
val mjvGLCamera_get_frustum_top : mjvGLCamera -> float

(** set frustum_top for mjvGLCamera *)
val mjvGLCamera_set_frustum_top : mjvGLCamera -> float -> unit

(** get frustum_near from mjvGLCamera *)
val mjvGLCamera_get_frustum_near : mjvGLCamera -> float

(** set frustum_near for mjvGLCamera *)
val mjvGLCamera_set_frustum_near : mjvGLCamera -> float -> unit

(** get frustum_far from mjvGLCamera *)
val mjvGLCamera_get_frustum_far : mjvGLCamera -> float

(** set frustum_far for mjvGLCamera *)
val mjvGLCamera_set_frustum_far : mjvGLCamera -> float -> unit

(** allocate fresh mjvGeom struct *)
val mjvGeom_allocate : unit -> mjvGeom

(** make null mjvGeom struct ptr *)
val mjvGeom_null : unit -> mjvGeom ptr

(** get type from mjvGeom *)
val mjvGeom_get_type : mjvGeom -> int

(** set type for mjvGeom *)
val mjvGeom_set_type : mjvGeom -> int -> unit

(** get dataid from mjvGeom *)
val mjvGeom_get_dataid : mjvGeom -> int

(** set dataid for mjvGeom *)
val mjvGeom_set_dataid : mjvGeom -> int -> unit

(** get objtype from mjvGeom *)
val mjvGeom_get_objtype : mjvGeom -> int

(** set objtype for mjvGeom *)
val mjvGeom_set_objtype : mjvGeom -> int -> unit

(** get objid from mjvGeom *)
val mjvGeom_get_objid : mjvGeom -> int

(** set objid for mjvGeom *)
val mjvGeom_set_objid : mjvGeom -> int -> unit

(** get category from mjvGeom *)
val mjvGeom_get_category : mjvGeom -> int

(** set category for mjvGeom *)
val mjvGeom_set_category : mjvGeom -> int -> unit

(** get texid from mjvGeom *)
val mjvGeom_get_texid : mjvGeom -> int

(** set texid for mjvGeom *)
val mjvGeom_set_texid : mjvGeom -> int -> unit

(** get texuniform from mjvGeom *)
val mjvGeom_get_texuniform : mjvGeom -> int

(** set texuniform for mjvGeom *)
val mjvGeom_set_texuniform : mjvGeom -> int -> unit

(** get texcoord from mjvGeom *)
val mjvGeom_get_texcoord : mjvGeom -> int

(** set texcoord for mjvGeom *)
val mjvGeom_set_texcoord : mjvGeom -> int -> unit

(** get segid from mjvGeom *)
val mjvGeom_get_segid : mjvGeom -> int

(** set segid for mjvGeom *)
val mjvGeom_set_segid : mjvGeom -> int -> unit

(** get texrepeat from mjvGeom *)
val mjvGeom_get_texrepeat : mjvGeom -> float ptr

(** set texrepeat for mjvGeom *)
val mjvGeom_set_texrepeat : mjvGeom -> float ptr -> unit

(** get size from mjvGeom *)
val mjvGeom_get_size : mjvGeom -> float ptr

(** set size for mjvGeom *)
val mjvGeom_set_size : mjvGeom -> float ptr -> unit

(** get pos from mjvGeom *)
val mjvGeom_get_pos : mjvGeom -> float ptr

(** set pos for mjvGeom *)
val mjvGeom_set_pos : mjvGeom -> float ptr -> unit

(** get mat from mjvGeom *)
val mjvGeom_get_mat : mjvGeom -> float ptr

(** set mat for mjvGeom *)
val mjvGeom_set_mat : mjvGeom -> float ptr -> unit

(** get rgba from mjvGeom *)
val mjvGeom_get_rgba : mjvGeom -> float ptr

(** set rgba for mjvGeom *)
val mjvGeom_set_rgba : mjvGeom -> float ptr -> unit

(** get emission from mjvGeom *)
val mjvGeom_get_emission : mjvGeom -> float

(** set emission for mjvGeom *)
val mjvGeom_set_emission : mjvGeom -> float -> unit

(** get specular from mjvGeom *)
val mjvGeom_get_specular : mjvGeom -> float

(** set specular for mjvGeom *)
val mjvGeom_set_specular : mjvGeom -> float -> unit

(** get shininess from mjvGeom *)
val mjvGeom_get_shininess : mjvGeom -> float

(** set shininess for mjvGeom *)
val mjvGeom_set_shininess : mjvGeom -> float -> unit

(** get reflectance from mjvGeom *)
val mjvGeom_get_reflectance : mjvGeom -> float

(** set reflectance for mjvGeom *)
val mjvGeom_set_reflectance : mjvGeom -> float -> unit

(** get label from mjvGeom *)
val mjvGeom_get_label : mjvGeom -> string

(** set label for mjvGeom *)
val mjvGeom_set_label : mjvGeom -> string -> unit

(** get camdist from mjvGeom *)
val mjvGeom_get_camdist : mjvGeom -> float

(** set camdist for mjvGeom *)
val mjvGeom_set_camdist : mjvGeom -> float -> unit

(** get modelrbound from mjvGeom *)
val mjvGeom_get_modelrbound : mjvGeom -> float

(** set modelrbound for mjvGeom *)
val mjvGeom_set_modelrbound : mjvGeom -> float -> unit

(** get transparent from mjvGeom *)
val mjvGeom_get_transparent : mjvGeom -> Unsigned.UChar.t

(** set transparent for mjvGeom *)
val mjvGeom_set_transparent : mjvGeom -> Unsigned.UChar.t -> unit

(** allocate fresh mjvLight struct *)
val mjvLight_allocate : unit -> mjvLight

(** make null mjvLight struct ptr *)
val mjvLight_null : unit -> mjvLight ptr

(** get pos from mjvLight *)
val mjvLight_get_pos : mjvLight -> float ptr

(** set pos for mjvLight *)
val mjvLight_set_pos : mjvLight -> float ptr -> unit

(** get dir from mjvLight *)
val mjvLight_get_dir : mjvLight -> float ptr

(** set dir for mjvLight *)
val mjvLight_set_dir : mjvLight -> float ptr -> unit

(** get attenuation from mjvLight *)
val mjvLight_get_attenuation : mjvLight -> float ptr

(** set attenuation for mjvLight *)
val mjvLight_set_attenuation : mjvLight -> float ptr -> unit

(** get cutoff from mjvLight *)
val mjvLight_get_cutoff : mjvLight -> float

(** set cutoff for mjvLight *)
val mjvLight_set_cutoff : mjvLight -> float -> unit

(** get exponent from mjvLight *)
val mjvLight_get_exponent : mjvLight -> float

(** set exponent for mjvLight *)
val mjvLight_set_exponent : mjvLight -> float -> unit

(** get ambient from mjvLight *)
val mjvLight_get_ambient : mjvLight -> float ptr

(** set ambient for mjvLight *)
val mjvLight_set_ambient : mjvLight -> float ptr -> unit

(** get diffuse from mjvLight *)
val mjvLight_get_diffuse : mjvLight -> float ptr

(** set diffuse for mjvLight *)
val mjvLight_set_diffuse : mjvLight -> float ptr -> unit

(** get specular from mjvLight *)
val mjvLight_get_specular : mjvLight -> float ptr

(** set specular for mjvLight *)
val mjvLight_set_specular : mjvLight -> float ptr -> unit

(** get headlight from mjvLight *)
val mjvLight_get_headlight : mjvLight -> Unsigned.UChar.t

(** set headlight for mjvLight *)
val mjvLight_set_headlight : mjvLight -> Unsigned.UChar.t -> unit

(** get directional from mjvLight *)
val mjvLight_get_directional : mjvLight -> Unsigned.UChar.t

(** set directional for mjvLight *)
val mjvLight_set_directional : mjvLight -> Unsigned.UChar.t -> unit

(** get castshadow from mjvLight *)
val mjvLight_get_castshadow : mjvLight -> Unsigned.UChar.t

(** set castshadow for mjvLight *)
val mjvLight_set_castshadow : mjvLight -> Unsigned.UChar.t -> unit

(** allocate fresh mjvOption struct *)
val mjvOption_allocate : unit -> mjvOption

(** make null mjvOption struct ptr *)
val mjvOption_null : unit -> mjvOption ptr

(** get label from mjvOption *)
val mjvOption_get_label : mjvOption -> int

(** set label for mjvOption *)
val mjvOption_set_label : mjvOption -> int -> unit

(** get frame from mjvOption *)
val mjvOption_get_frame : mjvOption -> int

(** set frame for mjvOption *)
val mjvOption_set_frame : mjvOption -> int -> unit

(** get geomgroup from mjvOption *)
val mjvOption_get_geomgroup : mjvOption -> Unsigned.UChar.t ptr

(** set geomgroup for mjvOption *)
val mjvOption_set_geomgroup : mjvOption -> Unsigned.UChar.t ptr -> unit

(** get sitegroup from mjvOption *)
val mjvOption_get_sitegroup : mjvOption -> Unsigned.UChar.t ptr

(** set sitegroup for mjvOption *)
val mjvOption_set_sitegroup : mjvOption -> Unsigned.UChar.t ptr -> unit

(** get jointgroup from mjvOption *)
val mjvOption_get_jointgroup : mjvOption -> Unsigned.UChar.t ptr

(** set jointgroup for mjvOption *)
val mjvOption_set_jointgroup : mjvOption -> Unsigned.UChar.t ptr -> unit

(** get tendongroup from mjvOption *)
val mjvOption_get_tendongroup : mjvOption -> Unsigned.UChar.t ptr

(** set tendongroup for mjvOption *)
val mjvOption_set_tendongroup : mjvOption -> Unsigned.UChar.t ptr -> unit

(** get actuatorgroup from mjvOption *)
val mjvOption_get_actuatorgroup : mjvOption -> Unsigned.UChar.t ptr

(** set actuatorgroup for mjvOption *)
val mjvOption_set_actuatorgroup : mjvOption -> Unsigned.UChar.t ptr -> unit

(** get flags from mjvOption *)
val mjvOption_get_flags : mjvOption -> Unsigned.UChar.t ptr

(** set flags for mjvOption *)
val mjvOption_set_flags : mjvOption -> Unsigned.UChar.t ptr -> unit

(** allocate fresh mjvScene struct *)
val mjvScene_allocate : unit -> mjvScene

(** make null mjvScene struct ptr *)
val mjvScene_null : unit -> mjvScene ptr

(** get maxgeom from mjvScene *)
val mjvScene_get_maxgeom : mjvScene -> int

(** set maxgeom for mjvScene *)
val mjvScene_set_maxgeom : mjvScene -> int -> unit

(** get ngeom from mjvScene *)
val mjvScene_get_ngeom : mjvScene -> int

(** set ngeom for mjvScene *)
val mjvScene_set_ngeom : mjvScene -> int -> unit

(** get geoms from mjvScene *)
val mjvScene_get_geoms : mjvScene -> mjvGeom ptr

(** set geoms for mjvScene *)
val mjvScene_set_geoms : mjvScene -> mjvGeom ptr -> unit

(** get geomorder from mjvScene *)
val mjvScene_get_geomorder : mjvScene -> int ptr

(** set geomorder for mjvScene *)
val mjvScene_set_geomorder : mjvScene -> int ptr -> unit

(** get nskin from mjvScene *)
val mjvScene_get_nskin : mjvScene -> int

(** set nskin for mjvScene *)
val mjvScene_set_nskin : mjvScene -> int -> unit

(** get skinfacenum from mjvScene *)
val mjvScene_get_skinfacenum : mjvScene -> int ptr

(** set skinfacenum for mjvScene *)
val mjvScene_set_skinfacenum : mjvScene -> int ptr -> unit

(** get skinvertadr from mjvScene *)
val mjvScene_get_skinvertadr : mjvScene -> int ptr

(** set skinvertadr for mjvScene *)
val mjvScene_set_skinvertadr : mjvScene -> int ptr -> unit

(** get skinvertnum from mjvScene *)
val mjvScene_get_skinvertnum : mjvScene -> int ptr

(** set skinvertnum for mjvScene *)
val mjvScene_set_skinvertnum : mjvScene -> int ptr -> unit

(** get skinvert from mjvScene *)
val mjvScene_get_skinvert : mjvScene -> float ptr

(** set skinvert for mjvScene *)
val mjvScene_set_skinvert : mjvScene -> float ptr -> unit

(** get skinnormal from mjvScene *)
val mjvScene_get_skinnormal : mjvScene -> float ptr

(** set skinnormal for mjvScene *)
val mjvScene_set_skinnormal : mjvScene -> float ptr -> unit

(** get nlight from mjvScene *)
val mjvScene_get_nlight : mjvScene -> int

(** set nlight for mjvScene *)
val mjvScene_set_nlight : mjvScene -> int -> unit

(** get lights from mjvScene *)
val mjvScene_get_lights : mjvScene -> mjvLight ptr

(** set lights for mjvScene *)
val mjvScene_set_lights : mjvScene -> mjvLight ptr -> unit

(** get camera from mjvScene *)
val mjvScene_get_camera : mjvScene -> mjvGLCamera ptr

(** set camera for mjvScene *)
val mjvScene_set_camera : mjvScene -> mjvGLCamera ptr -> unit

(** get enabletransform from mjvScene *)
val mjvScene_get_enabletransform : mjvScene -> Unsigned.UChar.t

(** set enabletransform for mjvScene *)
val mjvScene_set_enabletransform : mjvScene -> Unsigned.UChar.t -> unit

(** get translate from mjvScene *)
val mjvScene_get_translate : mjvScene -> float ptr

(** set translate for mjvScene *)
val mjvScene_set_translate : mjvScene -> float ptr -> unit

(** get rotate from mjvScene *)
val mjvScene_get_rotate : mjvScene -> float ptr

(** set rotate for mjvScene *)
val mjvScene_set_rotate : mjvScene -> float ptr -> unit

(** get scale from mjvScene *)
val mjvScene_get_scale : mjvScene -> float

(** set scale for mjvScene *)
val mjvScene_set_scale : mjvScene -> float -> unit

(** get stereo from mjvScene *)
val mjvScene_get_stereo : mjvScene -> int

(** set stereo for mjvScene *)
val mjvScene_set_stereo : mjvScene -> int -> unit

(** get flags from mjvScene *)
val mjvScene_get_flags : mjvScene -> Unsigned.UChar.t ptr

(** set flags for mjvScene *)
val mjvScene_set_flags : mjvScene -> Unsigned.UChar.t ptr -> unit

(** get framewidth from mjvScene *)
val mjvScene_get_framewidth : mjvScene -> int

(** set framewidth for mjvScene *)
val mjvScene_set_framewidth : mjvScene -> int -> unit

(** get framergb from mjvScene *)
val mjvScene_get_framergb : mjvScene -> float ptr

(** set framergb for mjvScene *)
val mjvScene_set_framergb : mjvScene -> float ptr -> unit

(** allocate fresh mjvFigure struct *)
val mjvFigure_allocate : unit -> mjvFigure

(** make null mjvFigure struct ptr *)
val mjvFigure_null : unit -> mjvFigure ptr

(** get flg_legend from mjvFigure *)
val mjvFigure_get_flg_legend : mjvFigure -> int

(** set flg_legend for mjvFigure *)
val mjvFigure_set_flg_legend : mjvFigure -> int -> unit

(** get flg_ticklabel from mjvFigure *)
val mjvFigure_get_flg_ticklabel : mjvFigure -> int ptr

(** set flg_ticklabel for mjvFigure *)
val mjvFigure_set_flg_ticklabel : mjvFigure -> int ptr -> unit

(** get flg_extend from mjvFigure *)
val mjvFigure_get_flg_extend : mjvFigure -> int

(** set flg_extend for mjvFigure *)
val mjvFigure_set_flg_extend : mjvFigure -> int -> unit

(** get flg_barplot from mjvFigure *)
val mjvFigure_get_flg_barplot : mjvFigure -> int

(** set flg_barplot for mjvFigure *)
val mjvFigure_set_flg_barplot : mjvFigure -> int -> unit

(** get flg_selection from mjvFigure *)
val mjvFigure_get_flg_selection : mjvFigure -> int

(** set flg_selection for mjvFigure *)
val mjvFigure_set_flg_selection : mjvFigure -> int -> unit

(** get flg_symmetric from mjvFigure *)
val mjvFigure_get_flg_symmetric : mjvFigure -> int

(** set flg_symmetric for mjvFigure *)
val mjvFigure_set_flg_symmetric : mjvFigure -> int -> unit

(** get linewidth from mjvFigure *)
val mjvFigure_get_linewidth : mjvFigure -> float

(** set linewidth for mjvFigure *)
val mjvFigure_set_linewidth : mjvFigure -> float -> unit

(** get gridwidth from mjvFigure *)
val mjvFigure_get_gridwidth : mjvFigure -> float

(** set gridwidth for mjvFigure *)
val mjvFigure_set_gridwidth : mjvFigure -> float -> unit

(** get gridsize from mjvFigure *)
val mjvFigure_get_gridsize : mjvFigure -> int ptr

(** set gridsize for mjvFigure *)
val mjvFigure_set_gridsize : mjvFigure -> int ptr -> unit

(** get gridrgb from mjvFigure *)
val mjvFigure_get_gridrgb : mjvFigure -> float ptr

(** set gridrgb for mjvFigure *)
val mjvFigure_set_gridrgb : mjvFigure -> float ptr -> unit

(** get figurergba from mjvFigure *)
val mjvFigure_get_figurergba : mjvFigure -> float ptr

(** set figurergba for mjvFigure *)
val mjvFigure_set_figurergba : mjvFigure -> float ptr -> unit

(** get panergba from mjvFigure *)
val mjvFigure_get_panergba : mjvFigure -> float ptr

(** set panergba for mjvFigure *)
val mjvFigure_set_panergba : mjvFigure -> float ptr -> unit

(** get legendrgba from mjvFigure *)
val mjvFigure_get_legendrgba : mjvFigure -> float ptr

(** set legendrgba for mjvFigure *)
val mjvFigure_set_legendrgba : mjvFigure -> float ptr -> unit

(** get textrgb from mjvFigure *)
val mjvFigure_get_textrgb : mjvFigure -> float ptr

(** set textrgb for mjvFigure *)
val mjvFigure_set_textrgb : mjvFigure -> float ptr -> unit

(** get xformat from mjvFigure *)
val mjvFigure_get_xformat : mjvFigure -> string

(** set xformat for mjvFigure *)
val mjvFigure_set_xformat : mjvFigure -> string -> unit

(** get yformat from mjvFigure *)
val mjvFigure_get_yformat : mjvFigure -> string

(** set yformat for mjvFigure *)
val mjvFigure_set_yformat : mjvFigure -> string -> unit

(** get minwidth from mjvFigure *)
val mjvFigure_get_minwidth : mjvFigure -> string

(** set minwidth for mjvFigure *)
val mjvFigure_set_minwidth : mjvFigure -> string -> unit

(** get title from mjvFigure *)
val mjvFigure_get_title : mjvFigure -> string

(** set title for mjvFigure *)
val mjvFigure_set_title : mjvFigure -> string -> unit

(** get xlabel from mjvFigure *)
val mjvFigure_get_xlabel : mjvFigure -> string

(** set xlabel for mjvFigure *)
val mjvFigure_set_xlabel : mjvFigure -> string -> unit

(** get legendoffset from mjvFigure *)
val mjvFigure_get_legendoffset : mjvFigure -> int

(** set legendoffset for mjvFigure *)
val mjvFigure_set_legendoffset : mjvFigure -> int -> unit

(** get subplot from mjvFigure *)
val mjvFigure_get_subplot : mjvFigure -> int

(** set subplot for mjvFigure *)
val mjvFigure_set_subplot : mjvFigure -> int -> unit

(** get highlight from mjvFigure *)
val mjvFigure_get_highlight : mjvFigure -> int ptr

(** set highlight for mjvFigure *)
val mjvFigure_set_highlight : mjvFigure -> int ptr -> unit

(** get highlightid from mjvFigure *)
val mjvFigure_get_highlightid : mjvFigure -> int

(** set highlightid for mjvFigure *)
val mjvFigure_set_highlightid : mjvFigure -> int -> unit

(** get selection from mjvFigure *)
val mjvFigure_get_selection : mjvFigure -> float

(** set selection for mjvFigure *)
val mjvFigure_set_selection : mjvFigure -> float -> unit

(** get linepnt from mjvFigure *)
val mjvFigure_get_linepnt : mjvFigure -> int ptr

(** set linepnt for mjvFigure *)
val mjvFigure_set_linepnt : mjvFigure -> int ptr -> unit

(** get xaxispixel from mjvFigure *)
val mjvFigure_get_xaxispixel : mjvFigure -> int ptr

(** set xaxispixel for mjvFigure *)
val mjvFigure_set_xaxispixel : mjvFigure -> int ptr -> unit

(** get yaxispixel from mjvFigure *)
val mjvFigure_get_yaxispixel : mjvFigure -> int ptr

(** set yaxispixel for mjvFigure *)
val mjvFigure_set_yaxispixel : mjvFigure -> int ptr -> unit

(** get xaxisdata from mjvFigure *)
val mjvFigure_get_xaxisdata : mjvFigure -> float ptr

(** set xaxisdata for mjvFigure *)
val mjvFigure_set_xaxisdata : mjvFigure -> float ptr -> unit

(** get yaxisdata from mjvFigure *)
val mjvFigure_get_yaxisdata : mjvFigure -> float ptr

(** set yaxisdata for mjvFigure *)
val mjvFigure_set_yaxisdata : mjvFigure -> float ptr -> unit

(** convert mjtGridPos type to int *)
val mjtGridPos_to_int : mjtGridPos -> int

(** convert mjtFramebuffer type to int *)
val mjtFramebuffer_to_int : mjtFramebuffer -> int

(** convert mjtFontScale type to int *)
val mjtFontScale_to_int : mjtFontScale -> int

(** convert mjtFont type to int *)
val mjtFont_to_int : mjtFont -> int

(** allocate fresh mjrRect struct *)
val mjrRect_allocate : unit -> mjrRect

(** make null mjrRect struct ptr *)
val mjrRect_null : unit -> mjrRect ptr

(** get left from mjrRect *)
val mjrRect_get_left : mjrRect -> int

(** set left for mjrRect *)
val mjrRect_set_left : mjrRect -> int -> unit

(** get bottom from mjrRect *)
val mjrRect_get_bottom : mjrRect -> int

(** set bottom for mjrRect *)
val mjrRect_set_bottom : mjrRect -> int -> unit

(** get width from mjrRect *)
val mjrRect_get_width : mjrRect -> int

(** set width for mjrRect *)
val mjrRect_set_width : mjrRect -> int -> unit

(** get height from mjrRect *)
val mjrRect_get_height : mjrRect -> int

(** set height for mjrRect *)
val mjrRect_set_height : mjrRect -> int -> unit

(** allocate fresh mjrContext struct *)
val mjrContext_allocate : unit -> mjrContext

(** make null mjrContext struct ptr *)
val mjrContext_null : unit -> mjrContext ptr

(** get lineWidth from mjrContext *)
val mjrContext_get_lineWidth : mjrContext -> float

(** set lineWidth for mjrContext *)
val mjrContext_set_lineWidth : mjrContext -> float -> unit

(** get shadowClip from mjrContext *)
val mjrContext_get_shadowClip : mjrContext -> float

(** set shadowClip for mjrContext *)
val mjrContext_set_shadowClip : mjrContext -> float -> unit

(** get shadowScale from mjrContext *)
val mjrContext_get_shadowScale : mjrContext -> float

(** set shadowScale for mjrContext *)
val mjrContext_set_shadowScale : mjrContext -> float -> unit

(** get fogStart from mjrContext *)
val mjrContext_get_fogStart : mjrContext -> float

(** set fogStart for mjrContext *)
val mjrContext_set_fogStart : mjrContext -> float -> unit

(** get fogEnd from mjrContext *)
val mjrContext_get_fogEnd : mjrContext -> float

(** set fogEnd for mjrContext *)
val mjrContext_set_fogEnd : mjrContext -> float -> unit

(** get fogRGBA from mjrContext *)
val mjrContext_get_fogRGBA : mjrContext -> float ptr

(** set fogRGBA for mjrContext *)
val mjrContext_set_fogRGBA : mjrContext -> float ptr -> unit

(** get shadowSize from mjrContext *)
val mjrContext_get_shadowSize : mjrContext -> int

(** set shadowSize for mjrContext *)
val mjrContext_set_shadowSize : mjrContext -> int -> unit

(** get offWidth from mjrContext *)
val mjrContext_get_offWidth : mjrContext -> int

(** set offWidth for mjrContext *)
val mjrContext_set_offWidth : mjrContext -> int -> unit

(** get offHeight from mjrContext *)
val mjrContext_get_offHeight : mjrContext -> int

(** set offHeight for mjrContext *)
val mjrContext_set_offHeight : mjrContext -> int -> unit

(** get offSamples from mjrContext *)
val mjrContext_get_offSamples : mjrContext -> int

(** set offSamples for mjrContext *)
val mjrContext_set_offSamples : mjrContext -> int -> unit

(** get fontScale from mjrContext *)
val mjrContext_get_fontScale : mjrContext -> int

(** set fontScale for mjrContext *)
val mjrContext_set_fontScale : mjrContext -> int -> unit

(** get auxWidth from mjrContext *)
val mjrContext_get_auxWidth : mjrContext -> int ptr

(** set auxWidth for mjrContext *)
val mjrContext_set_auxWidth : mjrContext -> int ptr -> unit

(** get auxHeight from mjrContext *)
val mjrContext_get_auxHeight : mjrContext -> int ptr

(** set auxHeight for mjrContext *)
val mjrContext_set_auxHeight : mjrContext -> int ptr -> unit

(** get auxSamples from mjrContext *)
val mjrContext_get_auxSamples : mjrContext -> int ptr

(** set auxSamples for mjrContext *)
val mjrContext_set_auxSamples : mjrContext -> int ptr -> unit

(** get ntexture from mjrContext *)
val mjrContext_get_ntexture : mjrContext -> int

(** set ntexture for mjrContext *)
val mjrContext_set_ntexture : mjrContext -> int -> unit

(** get textureType from mjrContext *)
val mjrContext_get_textureType : mjrContext -> int ptr

(** set textureType for mjrContext *)
val mjrContext_set_textureType : mjrContext -> int ptr -> unit

(** get rangePlane from mjrContext *)
val mjrContext_get_rangePlane : mjrContext -> int

(** set rangePlane for mjrContext *)
val mjrContext_set_rangePlane : mjrContext -> int -> unit

(** get rangeMesh from mjrContext *)
val mjrContext_get_rangeMesh : mjrContext -> int

(** set rangeMesh for mjrContext *)
val mjrContext_set_rangeMesh : mjrContext -> int -> unit

(** get rangeHField from mjrContext *)
val mjrContext_get_rangeHField : mjrContext -> int

(** set rangeHField for mjrContext *)
val mjrContext_set_rangeHField : mjrContext -> int -> unit

(** get rangeBuiltin from mjrContext *)
val mjrContext_get_rangeBuiltin : mjrContext -> int

(** set rangeBuiltin for mjrContext *)
val mjrContext_set_rangeBuiltin : mjrContext -> int -> unit

(** get rangeFont from mjrContext *)
val mjrContext_get_rangeFont : mjrContext -> int

(** set rangeFont for mjrContext *)
val mjrContext_set_rangeFont : mjrContext -> int -> unit

(** get nskin from mjrContext *)
val mjrContext_get_nskin : mjrContext -> int

(** set nskin for mjrContext *)
val mjrContext_set_nskin : mjrContext -> int -> unit

(** get charWidth from mjrContext *)
val mjrContext_get_charWidth : mjrContext -> int ptr

(** set charWidth for mjrContext *)
val mjrContext_set_charWidth : mjrContext -> int ptr -> unit

(** get charWidthBig from mjrContext *)
val mjrContext_get_charWidthBig : mjrContext -> int ptr

(** set charWidthBig for mjrContext *)
val mjrContext_set_charWidthBig : mjrContext -> int ptr -> unit

(** get charHeight from mjrContext *)
val mjrContext_get_charHeight : mjrContext -> int

(** set charHeight for mjrContext *)
val mjrContext_set_charHeight : mjrContext -> int -> unit

(** get charHeightBig from mjrContext *)
val mjrContext_get_charHeightBig : mjrContext -> int

(** set charHeightBig for mjrContext *)
val mjrContext_set_charHeightBig : mjrContext -> int -> unit

(** get glewInitialized from mjrContext *)
val mjrContext_get_glewInitialized : mjrContext -> int

(** set glewInitialized for mjrContext *)
val mjrContext_set_glewInitialized : mjrContext -> int -> unit

(** get windowAvailable from mjrContext *)
val mjrContext_get_windowAvailable : mjrContext -> int

(** set windowAvailable for mjrContext *)
val mjrContext_set_windowAvailable : mjrContext -> int -> unit

(** get windowSamples from mjrContext *)
val mjrContext_get_windowSamples : mjrContext -> int

(** set windowSamples for mjrContext *)
val mjrContext_set_windowSamples : mjrContext -> int -> unit

(** get windowStereo from mjrContext *)
val mjrContext_get_windowStereo : mjrContext -> int

(** set windowStereo for mjrContext *)
val mjrContext_set_windowStereo : mjrContext -> int -> unit

(** get windowDoublebuffer from mjrContext *)
val mjrContext_get_windowDoublebuffer : mjrContext -> int

(** set windowDoublebuffer for mjrContext *)
val mjrContext_set_windowDoublebuffer : mjrContext -> int -> unit

(** get currentBuffer from mjrContext *)
val mjrContext_get_currentBuffer : mjrContext -> int

(** set currentBuffer for mjrContext *)
val mjrContext_set_currentBuffer : mjrContext -> int -> unit

(** convert mjtButton type to int *)
val mjtButton_to_int : mjtButton -> int

(** convert mjtEvent type to int *)
val mjtEvent_to_int : mjtEvent -> int

(** convert mjtItem type to int *)
val mjtItem_to_int : mjtItem -> int

(** allocate fresh mjuiState struct *)
val mjuiState_allocate : unit -> mjuiState

(** make null mjuiState struct ptr *)
val mjuiState_null : unit -> mjuiState ptr

(** get nrect from mjuiState *)
val mjuiState_get_nrect : mjuiState -> int

(** set nrect for mjuiState *)
val mjuiState_set_nrect : mjuiState -> int -> unit

(** get rect from mjuiState *)
val mjuiState_get_rect : mjuiState -> mjrRect ptr

(** set rect for mjuiState *)
val mjuiState_set_rect : mjuiState -> mjrRect ptr -> unit

(** get userdata from mjuiState *)
val mjuiState_get_userdata : mjuiState -> unit ptr

(** set userdata for mjuiState *)
val mjuiState_set_userdata : mjuiState -> unit ptr -> unit

(** get type from mjuiState *)
val mjuiState_get_type : mjuiState -> int

(** set type for mjuiState *)
val mjuiState_set_type : mjuiState -> int -> unit

(** get left from mjuiState *)
val mjuiState_get_left : mjuiState -> int

(** set left for mjuiState *)
val mjuiState_set_left : mjuiState -> int -> unit

(** get right from mjuiState *)
val mjuiState_get_right : mjuiState -> int

(** set right for mjuiState *)
val mjuiState_set_right : mjuiState -> int -> unit

(** get middle from mjuiState *)
val mjuiState_get_middle : mjuiState -> int

(** set middle for mjuiState *)
val mjuiState_set_middle : mjuiState -> int -> unit

(** get doubleclick from mjuiState *)
val mjuiState_get_doubleclick : mjuiState -> int

(** set doubleclick for mjuiState *)
val mjuiState_set_doubleclick : mjuiState -> int -> unit

(** get button from mjuiState *)
val mjuiState_get_button : mjuiState -> int

(** set button for mjuiState *)
val mjuiState_set_button : mjuiState -> int -> unit

(** get buttontime from mjuiState *)
val mjuiState_get_buttontime : mjuiState -> float

(** set buttontime for mjuiState *)
val mjuiState_set_buttontime : mjuiState -> float -> unit

(** get x from mjuiState *)
val mjuiState_get_x : mjuiState -> float

(** set x for mjuiState *)
val mjuiState_set_x : mjuiState -> float -> unit

(** get y from mjuiState *)
val mjuiState_get_y : mjuiState -> float

(** set y for mjuiState *)
val mjuiState_set_y : mjuiState -> float -> unit

(** get dx from mjuiState *)
val mjuiState_get_dx : mjuiState -> float

(** set dx for mjuiState *)
val mjuiState_set_dx : mjuiState -> float -> unit

(** get dy from mjuiState *)
val mjuiState_get_dy : mjuiState -> float

(** set dy for mjuiState *)
val mjuiState_set_dy : mjuiState -> float -> unit

(** get sx from mjuiState *)
val mjuiState_get_sx : mjuiState -> float

(** set sx for mjuiState *)
val mjuiState_set_sx : mjuiState -> float -> unit

(** get sy from mjuiState *)
val mjuiState_get_sy : mjuiState -> float

(** set sy for mjuiState *)
val mjuiState_set_sy : mjuiState -> float -> unit

(** get control from mjuiState *)
val mjuiState_get_control : mjuiState -> int

(** set control for mjuiState *)
val mjuiState_set_control : mjuiState -> int -> unit

(** get shift from mjuiState *)
val mjuiState_get_shift : mjuiState -> int

(** set shift for mjuiState *)
val mjuiState_set_shift : mjuiState -> int -> unit

(** get alt from mjuiState *)
val mjuiState_get_alt : mjuiState -> int

(** set alt for mjuiState *)
val mjuiState_set_alt : mjuiState -> int -> unit

(** get key from mjuiState *)
val mjuiState_get_key : mjuiState -> int

(** set key for mjuiState *)
val mjuiState_set_key : mjuiState -> int -> unit

(** get keytime from mjuiState *)
val mjuiState_get_keytime : mjuiState -> float

(** set keytime for mjuiState *)
val mjuiState_set_keytime : mjuiState -> float -> unit

(** get mouserect from mjuiState *)
val mjuiState_get_mouserect : mjuiState -> int

(** set mouserect for mjuiState *)
val mjuiState_set_mouserect : mjuiState -> int -> unit

(** get dragrect from mjuiState *)
val mjuiState_get_dragrect : mjuiState -> int

(** set dragrect for mjuiState *)
val mjuiState_set_dragrect : mjuiState -> int -> unit

(** get dragbutton from mjuiState *)
val mjuiState_get_dragbutton : mjuiState -> int

(** set dragbutton for mjuiState *)
val mjuiState_set_dragbutton : mjuiState -> int -> unit

(** allocate fresh mjuiThemeSpacing struct *)
val mjuiThemeSpacing_allocate : unit -> mjuiThemeSpacing

(** make null mjuiThemeSpacing struct ptr *)
val mjuiThemeSpacing_null : unit -> mjuiThemeSpacing ptr

(** get total from mjuiThemeSpacing *)
val mjuiThemeSpacing_get_total : mjuiThemeSpacing -> int

(** set total for mjuiThemeSpacing *)
val mjuiThemeSpacing_set_total : mjuiThemeSpacing -> int -> unit

(** get scroll from mjuiThemeSpacing *)
val mjuiThemeSpacing_get_scroll : mjuiThemeSpacing -> int

(** set scroll for mjuiThemeSpacing *)
val mjuiThemeSpacing_set_scroll : mjuiThemeSpacing -> int -> unit

(** get label from mjuiThemeSpacing *)
val mjuiThemeSpacing_get_label : mjuiThemeSpacing -> int

(** set label for mjuiThemeSpacing *)
val mjuiThemeSpacing_set_label : mjuiThemeSpacing -> int -> unit

(** get section from mjuiThemeSpacing *)
val mjuiThemeSpacing_get_section : mjuiThemeSpacing -> int

(** set section for mjuiThemeSpacing *)
val mjuiThemeSpacing_set_section : mjuiThemeSpacing -> int -> unit

(** get itemside from mjuiThemeSpacing *)
val mjuiThemeSpacing_get_itemside : mjuiThemeSpacing -> int

(** set itemside for mjuiThemeSpacing *)
val mjuiThemeSpacing_set_itemside : mjuiThemeSpacing -> int -> unit

(** get itemmid from mjuiThemeSpacing *)
val mjuiThemeSpacing_get_itemmid : mjuiThemeSpacing -> int

(** set itemmid for mjuiThemeSpacing *)
val mjuiThemeSpacing_set_itemmid : mjuiThemeSpacing -> int -> unit

(** get itemver from mjuiThemeSpacing *)
val mjuiThemeSpacing_get_itemver : mjuiThemeSpacing -> int

(** set itemver for mjuiThemeSpacing *)
val mjuiThemeSpacing_set_itemver : mjuiThemeSpacing -> int -> unit

(** get texthor from mjuiThemeSpacing *)
val mjuiThemeSpacing_get_texthor : mjuiThemeSpacing -> int

(** set texthor for mjuiThemeSpacing *)
val mjuiThemeSpacing_set_texthor : mjuiThemeSpacing -> int -> unit

(** get textver from mjuiThemeSpacing *)
val mjuiThemeSpacing_get_textver : mjuiThemeSpacing -> int

(** set textver for mjuiThemeSpacing *)
val mjuiThemeSpacing_set_textver : mjuiThemeSpacing -> int -> unit

(** get linescroll from mjuiThemeSpacing *)
val mjuiThemeSpacing_get_linescroll : mjuiThemeSpacing -> int

(** set linescroll for mjuiThemeSpacing *)
val mjuiThemeSpacing_set_linescroll : mjuiThemeSpacing -> int -> unit

(** get samples from mjuiThemeSpacing *)
val mjuiThemeSpacing_get_samples : mjuiThemeSpacing -> int

(** set samples for mjuiThemeSpacing *)
val mjuiThemeSpacing_set_samples : mjuiThemeSpacing -> int -> unit

(** allocate fresh mjuiThemeColor struct *)
val mjuiThemeColor_allocate : unit -> mjuiThemeColor

(** make null mjuiThemeColor struct ptr *)
val mjuiThemeColor_null : unit -> mjuiThemeColor ptr

(** get master from mjuiThemeColor *)
val mjuiThemeColor_get_master : mjuiThemeColor -> float ptr

(** set master for mjuiThemeColor *)
val mjuiThemeColor_set_master : mjuiThemeColor -> float ptr -> unit

(** get thumb from mjuiThemeColor *)
val mjuiThemeColor_get_thumb : mjuiThemeColor -> float ptr

(** set thumb for mjuiThemeColor *)
val mjuiThemeColor_set_thumb : mjuiThemeColor -> float ptr -> unit

(** get secttitle from mjuiThemeColor *)
val mjuiThemeColor_get_secttitle : mjuiThemeColor -> float ptr

(** set secttitle for mjuiThemeColor *)
val mjuiThemeColor_set_secttitle : mjuiThemeColor -> float ptr -> unit

(** get sectfont from mjuiThemeColor *)
val mjuiThemeColor_get_sectfont : mjuiThemeColor -> float ptr

(** set sectfont for mjuiThemeColor *)
val mjuiThemeColor_set_sectfont : mjuiThemeColor -> float ptr -> unit

(** get sectsymbol from mjuiThemeColor *)
val mjuiThemeColor_get_sectsymbol : mjuiThemeColor -> float ptr

(** set sectsymbol for mjuiThemeColor *)
val mjuiThemeColor_set_sectsymbol : mjuiThemeColor -> float ptr -> unit

(** get sectpane from mjuiThemeColor *)
val mjuiThemeColor_get_sectpane : mjuiThemeColor -> float ptr

(** set sectpane for mjuiThemeColor *)
val mjuiThemeColor_set_sectpane : mjuiThemeColor -> float ptr -> unit

(** get shortcut from mjuiThemeColor *)
val mjuiThemeColor_get_shortcut : mjuiThemeColor -> float ptr

(** set shortcut for mjuiThemeColor *)
val mjuiThemeColor_set_shortcut : mjuiThemeColor -> float ptr -> unit

(** get fontactive from mjuiThemeColor *)
val mjuiThemeColor_get_fontactive : mjuiThemeColor -> float ptr

(** set fontactive for mjuiThemeColor *)
val mjuiThemeColor_set_fontactive : mjuiThemeColor -> float ptr -> unit

(** get fontinactive from mjuiThemeColor *)
val mjuiThemeColor_get_fontinactive : mjuiThemeColor -> float ptr

(** set fontinactive for mjuiThemeColor *)
val mjuiThemeColor_set_fontinactive : mjuiThemeColor -> float ptr -> unit

(** get decorinactive from mjuiThemeColor *)
val mjuiThemeColor_get_decorinactive : mjuiThemeColor -> float ptr

(** set decorinactive for mjuiThemeColor *)
val mjuiThemeColor_set_decorinactive : mjuiThemeColor -> float ptr -> unit

(** get decorinactive2 from mjuiThemeColor *)
val mjuiThemeColor_get_decorinactive2 : mjuiThemeColor -> float ptr

(** set decorinactive2 for mjuiThemeColor *)
val mjuiThemeColor_set_decorinactive2 : mjuiThemeColor -> float ptr -> unit

(** get button from mjuiThemeColor *)
val mjuiThemeColor_get_button : mjuiThemeColor -> float ptr

(** set button for mjuiThemeColor *)
val mjuiThemeColor_set_button : mjuiThemeColor -> float ptr -> unit

(** get check from mjuiThemeColor *)
val mjuiThemeColor_get_check : mjuiThemeColor -> float ptr

(** set check for mjuiThemeColor *)
val mjuiThemeColor_set_check : mjuiThemeColor -> float ptr -> unit

(** get radio from mjuiThemeColor *)
val mjuiThemeColor_get_radio : mjuiThemeColor -> float ptr

(** set radio for mjuiThemeColor *)
val mjuiThemeColor_set_radio : mjuiThemeColor -> float ptr -> unit

(** get select from mjuiThemeColor *)
val mjuiThemeColor_get_select : mjuiThemeColor -> float ptr

(** set select for mjuiThemeColor *)
val mjuiThemeColor_set_select : mjuiThemeColor -> float ptr -> unit

(** get select2 from mjuiThemeColor *)
val mjuiThemeColor_get_select2 : mjuiThemeColor -> float ptr

(** set select2 for mjuiThemeColor *)
val mjuiThemeColor_set_select2 : mjuiThemeColor -> float ptr -> unit

(** get slider from mjuiThemeColor *)
val mjuiThemeColor_get_slider : mjuiThemeColor -> float ptr

(** set slider for mjuiThemeColor *)
val mjuiThemeColor_set_slider : mjuiThemeColor -> float ptr -> unit

(** get slider2 from mjuiThemeColor *)
val mjuiThemeColor_get_slider2 : mjuiThemeColor -> float ptr

(** set slider2 for mjuiThemeColor *)
val mjuiThemeColor_set_slider2 : mjuiThemeColor -> float ptr -> unit

(** get edit from mjuiThemeColor *)
val mjuiThemeColor_get_edit : mjuiThemeColor -> float ptr

(** set edit for mjuiThemeColor *)
val mjuiThemeColor_set_edit : mjuiThemeColor -> float ptr -> unit

(** get edit2 from mjuiThemeColor *)
val mjuiThemeColor_get_edit2 : mjuiThemeColor -> float ptr

(** set edit2 for mjuiThemeColor *)
val mjuiThemeColor_set_edit2 : mjuiThemeColor -> float ptr -> unit

(** get cursor from mjuiThemeColor *)
val mjuiThemeColor_get_cursor : mjuiThemeColor -> float ptr

(** set cursor for mjuiThemeColor *)
val mjuiThemeColor_set_cursor : mjuiThemeColor -> float ptr -> unit

(** allocate fresh mjuiItemSingle struct *)
val mjuiItemSingle_allocate : unit -> mjuiItemSingle

(** make null mjuiItemSingle struct ptr *)
val mjuiItemSingle_null : unit -> mjuiItemSingle ptr

(** get modifier from mjuiItemSingle *)
val mjuiItemSingle_get_modifier : mjuiItemSingle -> int

(** set modifier for mjuiItemSingle *)
val mjuiItemSingle_set_modifier : mjuiItemSingle -> int -> unit

(** get shortcut from mjuiItemSingle *)
val mjuiItemSingle_get_shortcut : mjuiItemSingle -> int

(** set shortcut for mjuiItemSingle *)
val mjuiItemSingle_set_shortcut : mjuiItemSingle -> int -> unit

(** allocate fresh mjuiItemMulti struct *)
val mjuiItemMulti_allocate : unit -> mjuiItemMulti

(** make null mjuiItemMulti struct ptr *)
val mjuiItemMulti_null : unit -> mjuiItemMulti ptr

(** get nelem from mjuiItemMulti *)
val mjuiItemMulti_get_nelem : mjuiItemMulti -> int

(** set nelem for mjuiItemMulti *)
val mjuiItemMulti_set_nelem : mjuiItemMulti -> int -> unit

(** allocate fresh mjuiItemSlider struct *)
val mjuiItemSlider_allocate : unit -> mjuiItemSlider

(** make null mjuiItemSlider struct ptr *)
val mjuiItemSlider_null : unit -> mjuiItemSlider ptr

(** get range from mjuiItemSlider *)
val mjuiItemSlider_get_range : mjuiItemSlider -> float ptr

(** set range for mjuiItemSlider *)
val mjuiItemSlider_set_range : mjuiItemSlider -> float ptr -> unit

(** get divisions from mjuiItemSlider *)
val mjuiItemSlider_get_divisions : mjuiItemSlider -> float

(** set divisions for mjuiItemSlider *)
val mjuiItemSlider_set_divisions : mjuiItemSlider -> float -> unit

(** allocate fresh mjuiItemEdit struct *)
val mjuiItemEdit_allocate : unit -> mjuiItemEdit

(** make null mjuiItemEdit struct ptr *)
val mjuiItemEdit_null : unit -> mjuiItemEdit ptr

(** get nelem from mjuiItemEdit *)
val mjuiItemEdit_get_nelem : mjuiItemEdit -> int

(** set nelem for mjuiItemEdit *)
val mjuiItemEdit_set_nelem : mjuiItemEdit -> int -> unit

(** allocate fresh mjuiItem struct *)
val mjuiItem_allocate : unit -> mjuiItem

(** make null mjuiItem struct ptr *)
val mjuiItem_null : unit -> mjuiItem ptr

(** get type from mjuiItem *)
val mjuiItem_get_type : mjuiItem -> int

(** set type for mjuiItem *)
val mjuiItem_set_type : mjuiItem -> int -> unit

(** get name from mjuiItem *)
val mjuiItem_get_name : mjuiItem -> string

(** set name for mjuiItem *)
val mjuiItem_set_name : mjuiItem -> string -> unit

(** get state from mjuiItem *)
val mjuiItem_get_state : mjuiItem -> int

(** set state for mjuiItem *)
val mjuiItem_set_state : mjuiItem -> int -> unit

(** get sectionid from mjuiItem *)
val mjuiItem_get_sectionid : mjuiItem -> int

(** set sectionid for mjuiItem *)
val mjuiItem_set_sectionid : mjuiItem -> int -> unit

(** get itemid from mjuiItem *)
val mjuiItem_get_itemid : mjuiItem -> int

(** set itemid for mjuiItem *)
val mjuiItem_set_itemid : mjuiItem -> int -> unit

(** get rect from mjuiItem *)
val mjuiItem_get_rect : mjuiItem -> mjrRect

(** set rect for mjuiItem *)
val mjuiItem_set_rect : mjuiItem -> mjrRect -> unit

(** allocate fresh mjuiSection struct *)
val mjuiSection_allocate : unit -> mjuiSection

(** make null mjuiSection struct ptr *)
val mjuiSection_null : unit -> mjuiSection ptr

(** get name from mjuiSection *)
val mjuiSection_get_name : mjuiSection -> string

(** set name for mjuiSection *)
val mjuiSection_set_name : mjuiSection -> string -> unit

(** get state from mjuiSection *)
val mjuiSection_get_state : mjuiSection -> int

(** set state for mjuiSection *)
val mjuiSection_set_state : mjuiSection -> int -> unit

(** get modifier from mjuiSection *)
val mjuiSection_get_modifier : mjuiSection -> int

(** set modifier for mjuiSection *)
val mjuiSection_set_modifier : mjuiSection -> int -> unit

(** get shortcut from mjuiSection *)
val mjuiSection_get_shortcut : mjuiSection -> int

(** set shortcut for mjuiSection *)
val mjuiSection_set_shortcut : mjuiSection -> int -> unit

(** get nitem from mjuiSection *)
val mjuiSection_get_nitem : mjuiSection -> int

(** set nitem for mjuiSection *)
val mjuiSection_set_nitem : mjuiSection -> int -> unit

(** get item from mjuiSection *)
val mjuiSection_get_item : mjuiSection -> mjuiItem ptr

(** set item for mjuiSection *)
val mjuiSection_set_item : mjuiSection -> mjuiItem ptr -> unit

(** get rtitle from mjuiSection *)
val mjuiSection_get_rtitle : mjuiSection -> mjrRect

(** set rtitle for mjuiSection *)
val mjuiSection_set_rtitle : mjuiSection -> mjrRect -> unit

(** get rcontent from mjuiSection *)
val mjuiSection_get_rcontent : mjuiSection -> mjrRect

(** set rcontent for mjuiSection *)
val mjuiSection_set_rcontent : mjuiSection -> mjrRect -> unit

(** allocate fresh mjUI struct *)
val mjUI_allocate : unit -> mjUI

(** make null mjUI struct ptr *)
val mjUI_null : unit -> mjUI ptr

(** get spacing from mjUI *)
val mjUI_get_spacing : mjUI -> mjuiThemeSpacing

(** set spacing for mjUI *)
val mjUI_set_spacing : mjUI -> mjuiThemeSpacing -> unit

(** get color from mjUI *)
val mjUI_get_color : mjUI -> mjuiThemeColor

(** set color for mjUI *)
val mjUI_set_color : mjUI -> mjuiThemeColor -> unit

(** get predicate from mjUI *)
val mjUI_get_predicate : mjUI -> mjfItemEnable

(** set predicate for mjUI *)
val mjUI_set_predicate : mjUI -> mjfItemEnable -> unit

(** get userdata from mjUI *)
val mjUI_get_userdata : mjUI -> unit ptr

(** set userdata for mjUI *)
val mjUI_set_userdata : mjUI -> unit ptr -> unit

(** get rectid from mjUI *)
val mjUI_get_rectid : mjUI -> int

(** set rectid for mjUI *)
val mjUI_set_rectid : mjUI -> int -> unit

(** get auxid from mjUI *)
val mjUI_get_auxid : mjUI -> int

(** set auxid for mjUI *)
val mjUI_set_auxid : mjUI -> int -> unit

(** get radiocol from mjUI *)
val mjUI_get_radiocol : mjUI -> int

(** set radiocol for mjUI *)
val mjUI_set_radiocol : mjUI -> int -> unit

(** get width from mjUI *)
val mjUI_get_width : mjUI -> int

(** set width for mjUI *)
val mjUI_set_width : mjUI -> int -> unit

(** get height from mjUI *)
val mjUI_get_height : mjUI -> int

(** set height for mjUI *)
val mjUI_set_height : mjUI -> int -> unit

(** get maxheight from mjUI *)
val mjUI_get_maxheight : mjUI -> int

(** set maxheight for mjUI *)
val mjUI_set_maxheight : mjUI -> int -> unit

(** get scroll from mjUI *)
val mjUI_get_scroll : mjUI -> int

(** set scroll for mjUI *)
val mjUI_set_scroll : mjUI -> int -> unit

(** get mousesect from mjUI *)
val mjUI_get_mousesect : mjUI -> int

(** set mousesect for mjUI *)
val mjUI_set_mousesect : mjUI -> int -> unit

(** get mouseitem from mjUI *)
val mjUI_get_mouseitem : mjUI -> int

(** set mouseitem for mjUI *)
val mjUI_set_mouseitem : mjUI -> int -> unit

(** get mousehelp from mjUI *)
val mjUI_get_mousehelp : mjUI -> int

(** set mousehelp for mjUI *)
val mjUI_set_mousehelp : mjUI -> int -> unit

(** get editsect from mjUI *)
val mjUI_get_editsect : mjUI -> int

(** set editsect for mjUI *)
val mjUI_set_editsect : mjUI -> int -> unit

(** get edititem from mjUI *)
val mjUI_get_edititem : mjUI -> int

(** set edititem for mjUI *)
val mjUI_set_edititem : mjUI -> int -> unit

(** get editcursor from mjUI *)
val mjUI_get_editcursor : mjUI -> int

(** set editcursor for mjUI *)
val mjUI_set_editcursor : mjUI -> int -> unit

(** get editscroll from mjUI *)
val mjUI_get_editscroll : mjUI -> int

(** set editscroll for mjUI *)
val mjUI_set_editscroll : mjUI -> int -> unit

(** get edittext from mjUI *)
val mjUI_get_edittext : mjUI -> string

(** set edittext for mjUI *)
val mjUI_set_edittext : mjUI -> string -> unit

(** get editchanged from mjUI *)
val mjUI_get_editchanged : mjUI -> mjuiItem ptr

(** set editchanged for mjUI *)
val mjUI_set_editchanged : mjUI -> mjuiItem ptr -> unit

(** get nsect from mjUI *)
val mjUI_get_nsect : mjUI -> int

(** set nsect for mjUI *)
val mjUI_set_nsect : mjUI -> int -> unit

(** get sect from mjUI *)
val mjUI_get_sect : mjUI -> mjuiSection ptr

(** set sect for mjUI *)
val mjUI_set_sect : mjUI -> mjuiSection ptr -> unit

(** allocate fresh mjuiDef struct *)
val mjuiDef_allocate : unit -> mjuiDef

(** make null mjuiDef struct ptr *)
val mjuiDef_null : unit -> mjuiDef ptr

(** get type from mjuiDef *)
val mjuiDef_get_type : mjuiDef -> int

(** set type for mjuiDef *)
val mjuiDef_set_type : mjuiDef -> int -> unit

(** get name from mjuiDef *)
val mjuiDef_get_name : mjuiDef -> string

(** set name for mjuiDef *)
val mjuiDef_set_name : mjuiDef -> string -> unit

(** get state from mjuiDef *)
val mjuiDef_get_state : mjuiDef -> int

(** set state for mjuiDef *)
val mjuiDef_set_state : mjuiDef -> int -> unit

(** get pdata from mjuiDef *)
val mjuiDef_get_pdata : mjuiDef -> unit ptr

(** set pdata for mjuiDef *)
val mjuiDef_set_pdata : mjuiDef -> unit ptr -> unit

(** get other from mjuiDef *)
val mjuiDef_get_other : mjuiDef -> string

(** set other for mjuiDef *)
val mjuiDef_set_other : mjuiDef -> string -> unit

(** Return 1 (for backward compatibility). *)
val mj_activate : string -> int

(** Do nothing (for backward compatibility). *)
val mj_deactivate : unit -> unit

(** Initialize VFS to empty (no deallocation). *)
val mj_defaultVFS : mjVFS ptr -> unit

(** Add file to VFS, return 0: success, 1: full, 2: repeated name, -1: not found on disk. *)
val mj_addFileVFS : mjVFS ptr -> string -> string -> int

(** Make empty file in VFS, return 0: success, 1: full, 2: repeated name. *)
val mj_makeEmptyFileVFS : mjVFS ptr -> string -> int -> int

(** Return file index in VFS, or -1 if not found in VFS. *)
val mj_findFileVFS : mjVFS ptr -> string -> int

(** Delete file from VFS, return 0: success, -1: not found in VFS. *)
val mj_deleteFileVFS : mjVFS ptr -> string -> int

(** Delete all files from VFS. *)
val mj_deleteVFS : mjVFS ptr -> unit

(** Parse XML file in MJCF or URDF format, compile it, return low-level model.
 If vfs is not NULL, look up files in vfs before reading from disk.
 If error is not NULL, it must have size error_sz. *)
val mj_loadXML : string -> mjVFS ptr -> string -> int -> mjModel ptr

(** Update XML data structures with info from low-level model, save as MJCF.
 If error is not NULL, it must have size error_sz. *)
val mj_saveLastXML : string -> mjModel ptr -> string -> int -> int

(** Free last XML model if loaded. Called internally at each load. *)
val mj_freeLastXML : unit -> unit

(** Print internal XML schema as plain text or HTML, with style-padding or &nbsp;. *)
val mj_printSchema : string -> string -> int -> int -> int -> int

(** Advance simulation, use control callback to obtain external force and control. *)
val mj_step : mjModel ptr -> mjData ptr -> unit

(** Advance simulation in two steps: before external force and control is set by user. *)
val mj_step1 : mjModel ptr -> mjData ptr -> unit

(** Advance simulation in two steps: after external force and control is set by user. *)
val mj_step2 : mjModel ptr -> mjData ptr -> unit

(** Forward dynamics: same as mj_step but do not integrate in time. *)
val mj_forward : mjModel ptr -> mjData ptr -> unit

(** Inverse dynamics: qacc must be set before calling. *)
val mj_inverse : mjModel ptr -> mjData ptr -> unit

(** Forward dynamics with skip; skipstage is mjtStage. *)
val mj_forwardSkip : mjModel ptr -> mjData ptr -> int -> int -> unit

(** Inverse dynamics with skip; skipstage is mjtStage. *)
val mj_inverseSkip : mjModel ptr -> mjData ptr -> int -> int -> unit

(** Set default options for length range computation. *)
val mj_defaultLROpt : mjLROpt ptr -> unit

(** Set solver parameters to default values. *)
val mj_defaultSolRefImp : float ptr -> float ptr -> unit

(** Set physics options to default values. *)
val mj_defaultOption : mjOption ptr -> unit

(** Set visual options to default values. *)
val mj_defaultVisual : mjVisual ptr -> unit

(** Copy mjModel, allocate new if dest is NULL. *)
val mj_copyModel : mjModel ptr -> mjModel ptr -> mjModel ptr

(** Save model to binary MJB file or memory buffer; buffer has precedence when given. *)
val mj_saveModel : mjModel ptr -> string -> unit ptr -> int -> unit

(** Load model from binary MJB file.
 If vfs is not NULL, look up file in vfs before reading from disk. *)
val mj_loadModel : string -> mjVFS ptr -> mjModel ptr

(** Free memory allocation in model. *)
val mj_deleteModel : mjModel ptr -> unit

(** Return size of buffer needed to hold model. *)
val mj_sizeModel : mjModel ptr -> int

(** Allocate mjData correponding to given model. *)
val mj_makeData : mjModel ptr -> mjData ptr

(** Copy mjData. *)
val mj_copyData : mjData ptr -> mjModel ptr -> mjData ptr -> mjData ptr

(** Reset data to defaults. *)
val mj_resetData : mjModel ptr -> mjData ptr -> unit

(** Reset data to defaults, fill everything else with debug_value. *)
val mj_resetDataDebug : mjModel ptr -> mjData ptr -> Unsigned.UChar.t -> unit

(** Reset data, set fields from specified keyframe. *)
val mj_resetDataKeyframe : mjModel ptr -> mjData ptr -> int -> unit

(** Allocate array of specified size on mjData stack. Call mju_error on stack overflow. *)
val mj_stackAlloc : mjData ptr -> int -> float ptr

(** Free memory allocation in mjData. *)
val mj_deleteData : mjData ptr -> unit

(** Reset all callbacks to NULL pointers (NULL is the default). *)
val mj_resetCallbacks : unit -> unit

(** Set constant fields of mjModel, corresponding to qpos0 configuration. *)
val mj_setConst : mjModel ptr -> mjData ptr -> unit

(** Set actuator_lengthrange for specified actuator; return 1 if ok, 0 if error. *)
val mj_setLengthRange
  :  mjModel ptr
  -> mjData ptr
  -> int
  -> mjLROpt ptr
  -> string
  -> int
  -> int

(** Print model to text file. *)
val mj_printModel : mjModel ptr -> string -> unit

(** Print data to text file. *)
val mj_printData : mjModel ptr -> mjData ptr -> string -> unit

(** Print matrix to screen. *)
val mju_printMat : float ptr -> int -> int -> unit

(** Print sparse matrix to screen. *)
val mju_printMatSparse : float ptr -> int -> int ptr -> int ptr -> int ptr -> unit

(** Run position-dependent computations. *)
val mj_fwdPosition : mjModel ptr -> mjData ptr -> unit

(** Run velocity-dependent computations. *)
val mj_fwdVelocity : mjModel ptr -> mjData ptr -> unit

(** Compute actuator force qfrc_actuation. *)
val mj_fwdActuation : mjModel ptr -> mjData ptr -> unit

(** Add up all non-constraint forces, compute qacc_unc. *)
val mj_fwdAcceleration : mjModel ptr -> mjData ptr -> unit

(** Run selected constraint solver. *)
val mj_fwdConstraint : mjModel ptr -> mjData ptr -> unit

(** Euler integrator, semi-implicit in velocity. *)
val mj_Euler : mjModel ptr -> mjData ptr -> unit

(** Runge-Kutta explicit order-N integrator. *)
val mj_RungeKutta : mjModel ptr -> mjData ptr -> int -> unit

(** Run position-dependent computations in inverse dynamics. *)
val mj_invPosition : mjModel ptr -> mjData ptr -> unit

(** Run velocity-dependent computations in inverse dynamics. *)
val mj_invVelocity : mjModel ptr -> mjData ptr -> unit

(** Apply the analytical formula for inverse constraint dynamics. *)
val mj_invConstraint : mjModel ptr -> mjData ptr -> unit

(** Compare forward and inverse dynamics, save results in fwdinv. *)
val mj_compareFwdInv : mjModel ptr -> mjData ptr -> unit

(** Evaluate position-dependent sensors. *)
val mj_sensorPos : mjModel ptr -> mjData ptr -> unit

(** Evaluate velocity-dependent sensors. *)
val mj_sensorVel : mjModel ptr -> mjData ptr -> unit

(** Evaluate acceleration and force-dependent sensors. *)
val mj_sensorAcc : mjModel ptr -> mjData ptr -> unit

(** Evaluate position-dependent energy (potential). *)
val mj_energyPos : mjModel ptr -> mjData ptr -> unit

(** Evaluate velocity-dependent energy (kinetic). *)
val mj_energyVel : mjModel ptr -> mjData ptr -> unit

(** Check qpos, reset if any element is too big or nan. *)
val mj_checkPos : mjModel ptr -> mjData ptr -> unit

(** Check qvel, reset if any element is too big or nan. *)
val mj_checkVel : mjModel ptr -> mjData ptr -> unit

(** Check qacc, reset if any element is too big or nan. *)
val mj_checkAcc : mjModel ptr -> mjData ptr -> unit

(** Run forward kinematics. *)
val mj_kinematics : mjModel ptr -> mjData ptr -> unit

(** Map inertias and motion dofs to global frame centered at CoM. *)
val mj_comPos : mjModel ptr -> mjData ptr -> unit

(** Compute camera and light positions and orientations. *)
val mj_camlight : mjModel ptr -> mjData ptr -> unit

(** Compute tendon lengths, velocities and moment arms. *)
val mj_tendon : mjModel ptr -> mjData ptr -> unit

(** Compute actuator transmission lengths and moments. *)
val mj_transmission : mjModel ptr -> mjData ptr -> unit

(** Run composite rigid body inertia algorithm (CRB). *)
val mj_crb : mjModel ptr -> mjData ptr -> unit

(** Compute sparse L'*D*L factorizaton of inertia matrix. *)
val mj_factorM : mjModel ptr -> mjData ptr -> unit

(** Solve linear system M * x = y using factorization:  x = inv(L'*D*L)*y *)
val mj_solveM : mjModel ptr -> mjData ptr -> float ptr -> float ptr -> int -> unit

(** Half of linear solve:  x = sqrt(inv(D))*inv(L')*y *)
val mj_solveM2 : mjModel ptr -> mjData ptr -> float ptr -> float ptr -> int -> unit

(** Compute cvel, cdof_dot. *)
val mj_comVel : mjModel ptr -> mjData ptr -> unit

(** Compute qfrc_passive from spring-dampers, viscosity and density. *)
val mj_passive : mjModel ptr -> mjData ptr -> unit

(** subtree linear velocity and angular momentum *)
val mj_subtreeVel : mjModel ptr -> mjData ptr -> unit

(** RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term. *)
val mj_rne : mjModel ptr -> mjData ptr -> int -> float ptr -> unit

(** RNE with complete data: compute cacc, cfrc_ext, cfrc_int. *)
val mj_rnePostConstraint : mjModel ptr -> mjData ptr -> unit

(** Run collision detection. *)
val mj_collision : mjModel ptr -> mjData ptr -> unit

(** Construct constraints. *)
val mj_makeConstraint : mjModel ptr -> mjData ptr -> unit

(** Compute inverse constaint inertia efc_AR. *)
val mj_projectConstraint : mjModel ptr -> mjData ptr -> unit

(** Compute efc_vel, efc_aref. *)
val mj_referenceConstraint : mjModel ptr -> mjData ptr -> unit

(** Compute efc_state, efc_force, qfrc_constraint, and (optionally) cone Hessians.
 If cost is not NULL, set *cost = s(jar) where jar = Jac*qacc-aref. *)
val mj_constraintUpdate
  :  mjModel ptr
  -> mjData ptr
  -> float ptr
  -> float ptr
  -> int
  -> unit

(** Add contact to d->contact list; return 0 if success; 1 if buffer full. *)
val mj_addContact : mjModel ptr -> mjData ptr -> mjContact ptr -> int

(** Determine type of friction cone. *)
val mj_isPyramidal : mjModel ptr -> int

(** Determine type of constraint Jacobian. *)
val mj_isSparse : mjModel ptr -> int

(** Determine type of solver (PGS is dual, CG and Newton are primal). *)
val mj_isDual : mjModel ptr -> int

(** Multiply dense or sparse constraint Jacobian by vector. *)
val mj_mulJacVec : mjModel ptr -> mjData ptr -> float ptr -> float ptr -> unit

(** Multiply dense or sparse constraint Jacobian transpose by vector. *)
val mj_mulJacTVec : mjModel ptr -> mjData ptr -> float ptr -> float ptr -> unit

(** Compute 3/6-by-nv end-effector Jacobian of global point attached to given body. *)
val mj_jac
  :  mjModel ptr
  -> mjData ptr
  -> float ptr
  -> float ptr
  -> float ptr
  -> int
  -> unit

(** Compute body frame end-effector Jacobian. *)
val mj_jacBody : mjModel ptr -> mjData ptr -> float ptr -> float ptr -> int -> unit

(** Compute body center-of-mass end-effector Jacobian. *)
val mj_jacBodyCom : mjModel ptr -> mjData ptr -> float ptr -> float ptr -> int -> unit

(** Compute geom end-effector Jacobian. *)
val mj_jacGeom : mjModel ptr -> mjData ptr -> float ptr -> float ptr -> int -> unit

(** Compute site end-effector Jacobian. *)
val mj_jacSite : mjModel ptr -> mjData ptr -> float ptr -> float ptr -> int -> unit

(** Compute translation end-effector Jacobian of point, and rotation Jacobian of axis. *)
val mj_jacPointAxis
  :  mjModel ptr
  -> mjData ptr
  -> float ptr
  -> float ptr
  -> float ptr
  -> float ptr
  -> int
  -> unit

(** Get id of object with specified name, return -1 if not found; type is mjtObj. *)
val mj_name2id : mjModel ptr -> int -> string -> int

(** Convert sparse inertia matrix M into full (i.e. dense) matrix. *)
val mj_fullM : mjModel ptr -> float ptr -> float ptr -> unit

(** Multiply vector by inertia matrix. *)
val mj_mulM : mjModel ptr -> mjData ptr -> float ptr -> float ptr -> unit

(** Multiply vector by (inertia matrix)^(1/2). *)
val mj_mulM2 : mjModel ptr -> mjData ptr -> float ptr -> float ptr -> unit

(** Add inertia matrix to destination matrix.
 Destination can be sparse uncompressed, or dense when all int* are NULL *)
val mj_addM
  :  mjModel ptr
  -> mjData ptr
  -> float ptr
  -> int ptr
  -> int ptr
  -> int ptr
  -> unit

(** Apply cartesian force and torque (outside xfrc_applied mechanism). *)
val mj_applyFT
  :  mjModel ptr
  -> mjData ptr
  -> float ptr
  -> float ptr
  -> float ptr
  -> int
  -> float ptr
  -> unit

(** Compute object 6D velocity in object-centered frame, world/local orientation. *)
val mj_objectVelocity
  :  mjModel ptr
  -> mjData ptr
  -> int
  -> int
  -> float ptr
  -> int
  -> unit

(** Compute object 6D acceleration in object-centered frame, world/local orientation. *)
val mj_objectAcceleration
  :  mjModel ptr
  -> mjData ptr
  -> int
  -> int
  -> float ptr
  -> int
  -> unit

(** Extract 6D force:torque for one contact, in contact frame. *)
val mj_contactForce : mjModel ptr -> mjData ptr -> int -> float ptr -> unit

(** Compute velocity by finite-differencing two positions. *)
val mj_differentiatePos
  :  mjModel ptr
  -> float ptr
  -> float
  -> float ptr
  -> float ptr
  -> unit

(** Integrate position with given velocity. *)
val mj_integratePos : mjModel ptr -> float ptr -> float ptr -> float -> unit

(** Normalize all quaterions in qpos-type vector. *)
val mj_normalizeQuat : mjModel ptr -> float ptr -> unit

(** Map from body local to global Cartesian coordinates. *)
val mj_local2Global
  :  mjData ptr
  -> float ptr
  -> float ptr
  -> float ptr
  -> float ptr
  -> int
  -> Unsigned.UChar.t
  -> unit

(** Sum all body masses. *)
val mj_getTotalmass : mjModel ptr -> float

(** Scale body masses and inertias to achieve specified total mass. *)
val mj_setTotalmass : mjModel ptr -> float -> unit

(** Return version number: 1.0.2 is encoded as 102. *)
val mj_version : unit -> int

(** Intersect ray (pnt+x*vec, x>=0) with visible geoms, except geoms in bodyexclude.
 Return geomid and distance (x) to nearest surface, or -1 if no intersection.
 geomgroup, flg_static are as in mjvOption; geomgroup==NULL skips group exclusion. *)
val mj_ray
  :  mjModel ptr
  -> mjData ptr
  -> float ptr
  -> float ptr
  -> Unsigned.UChar.t ptr
  -> Unsigned.UChar.t
  -> int
  -> int ptr
  -> float

(** Interect ray with hfield, return nearest distance or -1 if no intersection. *)
val mj_rayHfield : mjModel ptr -> mjData ptr -> int -> float ptr -> float ptr -> float

(** Interect ray with mesh, return nearest distance or -1 if no intersection. *)
val mj_rayMesh : mjModel ptr -> mjData ptr -> int -> float ptr -> float ptr -> float

(** Interect ray with pure geom, return nearest distance or -1 if no intersection. *)
val mju_rayGeom
  :  float ptr
  -> float ptr
  -> float ptr
  -> float ptr
  -> float ptr
  -> int
  -> float

(** Interect ray with skin, return nearest vertex id. *)
val mju_raySkin
  :  int
  -> int
  -> int ptr
  -> float ptr
  -> float ptr
  -> float ptr
  -> int ptr
  -> float

(** Set default camera. *)
val mjv_defaultCamera : mjvCamera ptr -> unit

(** Set default perturbation. *)
val mjv_defaultPerturb : mjvPerturb ptr -> unit

(** Transform pose from room to model space. *)
val mjv_room2model
  :  float ptr
  -> float ptr
  -> float ptr
  -> float ptr
  -> mjvScene ptr
  -> unit

(** Transform pose from model to room space. *)
val mjv_model2room
  :  float ptr
  -> float ptr
  -> float ptr
  -> float ptr
  -> mjvScene ptr
  -> unit

(** Get camera info in model space; average left and right OpenGL cameras. *)
val mjv_cameraInModel : float ptr -> float ptr -> float ptr -> mjvScene ptr -> unit

(** Get camera info in room space; average left and right OpenGL cameras. *)
val mjv_cameraInRoom : float ptr -> float ptr -> float ptr -> mjvScene ptr -> unit

(** Get frustum height at unit distance from camera; average left and right OpenGL cameras. *)
val mjv_frustumHeight : mjvScene ptr -> float

(** Rotate 3D vec in horizontal plane by angle between (0,1) and (forward_x,forward_y). *)
val mjv_alignToCamera : float ptr -> float ptr -> float ptr -> unit

(** Move camera with mouse; action is mjtMouse. *)
val mjv_moveCamera
  :  mjModel ptr
  -> int
  -> float
  -> float
  -> mjvScene ptr
  -> mjvCamera ptr
  -> unit

(** Move perturb object with mouse; action is mjtMouse. *)
val mjv_movePerturb
  :  mjModel ptr
  -> mjData ptr
  -> int
  -> float
  -> float
  -> mjvScene ptr
  -> mjvPerturb ptr
  -> unit

(** Move model with mouse; action is mjtMouse. *)
val mjv_moveModel
  :  mjModel ptr
  -> int
  -> float
  -> float
  -> float ptr
  -> mjvScene ptr
  -> unit

(** Copy perturb pos,quat from selected body; set scale for perturbation. *)
val mjv_initPerturb : mjModel ptr -> mjData ptr -> mjvScene ptr -> mjvPerturb ptr -> unit

(** Set perturb pos,quat in d->mocap when selected body is mocap, and in d->qpos otherwise.
 Write d->qpos only if flg_paused and subtree root for selected body has free joint. *)
val mjv_applyPerturbPose : mjModel ptr -> mjData ptr -> mjvPerturb ptr -> int -> unit

(** Set perturb force,torque in d->xfrc_applied, if selected body is dynamic. *)
val mjv_applyPerturbForce : mjModel ptr -> mjData ptr -> mjvPerturb ptr -> unit

(** Return the average of two OpenGL cameras. *)
val mjv_averageCamera : mjvGLCamera ptr -> mjvGLCamera ptr -> mjvGLCamera

(** Select geom or skin with mouse, return bodyid; -1: none selected. *)
val mjv_select
  :  mjModel ptr
  -> mjData ptr
  -> mjvOption ptr
  -> float
  -> float
  -> float
  -> mjvScene ptr
  -> float ptr
  -> int ptr
  -> int ptr
  -> int

(** Set default visualization options. *)
val mjv_defaultOption : mjvOption ptr -> unit

(** Set default figure. *)
val mjv_defaultFigure : mjvFigure ptr -> unit

(** Initialize given geom fields when not NULL, set the rest to their default values. *)
val mjv_initGeom
  :  mjvGeom ptr
  -> int
  -> float ptr
  -> float ptr
  -> float ptr
  -> float ptr
  -> unit

(** Set (type, size, pos, mat) for connector-type geom between given points.
 Assume that mjv_initGeom was already called to set all other properties. *)
val mjv_makeConnector
  :  mjvGeom ptr
  -> int
  -> float
  -> float
  -> float
  -> float
  -> float
  -> float
  -> float
  -> unit

(** Set default abstract scene. *)
val mjv_defaultScene : mjvScene ptr -> unit

(** Allocate resources in abstract scene. *)
val mjv_makeScene : mjModel ptr -> mjvScene ptr -> int -> unit

(** Free abstract scene. *)
val mjv_freeScene : mjvScene ptr -> unit

(** Update entire scene given model state. *)
val mjv_updateScene
  :  mjModel ptr
  -> mjData ptr
  -> mjvOption ptr
  -> mjvPerturb ptr
  -> mjvCamera ptr
  -> int
  -> mjvScene ptr
  -> unit

(** Add geoms from selected categories. *)
val mjv_addGeoms
  :  mjModel ptr
  -> mjData ptr
  -> mjvOption ptr
  -> mjvPerturb ptr
  -> int
  -> mjvScene ptr
  -> unit

(** Make list of lights. *)
val mjv_makeLights : mjModel ptr -> mjData ptr -> mjvScene ptr -> unit

(** Update camera. *)
val mjv_updateCamera : mjModel ptr -> mjData ptr -> mjvCamera ptr -> mjvScene ptr -> unit

(** Update skins. *)
val mjv_updateSkin : mjModel ptr -> mjData ptr -> mjvScene ptr -> unit

(** Set default mjrContext. *)
val mjr_defaultContext : mjrContext ptr -> unit

(** Allocate resources in custom OpenGL context; fontscale is mjtFontScale. *)
val mjr_makeContext : mjModel ptr -> mjrContext ptr -> int -> unit

(** Change font of existing context. *)
val mjr_changeFont : int -> mjrContext ptr -> unit

(** Add Aux buffer with given index to context; free previous Aux buffer. *)
val mjr_addAux : int -> int -> int -> int -> mjrContext ptr -> unit

(** Free resources in custom OpenGL context, set to default. *)
val mjr_freeContext : mjrContext ptr -> unit

(** Upload texture to GPU, overwriting previous upload if any. *)
val mjr_uploadTexture : mjModel ptr -> mjrContext ptr -> int -> unit

(** Upload mesh to GPU, overwriting previous upload if any. *)
val mjr_uploadMesh : mjModel ptr -> mjrContext ptr -> int -> unit

(** Upload height field to GPU, overwriting previous upload if any. *)
val mjr_uploadHField : mjModel ptr -> mjrContext ptr -> int -> unit

(** Make con->currentBuffer current again. *)
val mjr_restoreBuffer : mjrContext ptr -> unit

(** Set OpenGL framebuffer for rendering: mjFB_WINDOW or mjFB_OFFSCREEN.
 If only one buffer is available, set that buffer and ignore framebuffer argument. *)
val mjr_setBuffer : int -> mjrContext ptr -> unit

(** Read pixels from current OpenGL framebuffer to client buffer.
 Viewport is in OpenGL framebuffer; client buffer starts at (0,0). *)
val mjr_readPixels
  :  Unsigned.UChar.t ptr
  -> float ptr
  -> mjrRect
  -> mjrContext ptr
  -> unit

(** Draw pixels from client buffer to current OpenGL framebuffer.
 Viewport is in OpenGL framebuffer; client buffer starts at (0,0). *)
val mjr_drawPixels
  :  Unsigned.UChar.t ptr
  -> float ptr
  -> mjrRect
  -> mjrContext ptr
  -> unit

(** Blit from src viewpoint in current framebuffer to dst viewport in other framebuffer.
 If src, dst have different size and flg_depth==0, color is interpolated with GL_LINEAR. *)
val mjr_blitBuffer : mjrRect -> mjrRect -> int -> int -> mjrContext ptr -> unit

(** Set Aux buffer for custom OpenGL rendering (call restoreBuffer when done). *)
val mjr_setAux : int -> mjrContext ptr -> unit

(** Blit from Aux buffer to con->currentBuffer. *)
val mjr_blitAux : int -> mjrRect -> int -> int -> mjrContext ptr -> unit

(** Draw text at (x,y) in relative coordinates; font is mjtFont. *)
val mjr_text
  :  int
  -> string
  -> mjrContext ptr
  -> float
  -> float
  -> float
  -> float
  -> float
  -> unit

(** Draw text overlay; font is mjtFont; gridpos is mjtGridPos. *)
val mjr_overlay : int -> int -> mjrRect -> string -> string -> mjrContext ptr -> unit

(** Get maximum viewport for active buffer. *)
val mjr_maxViewport : mjrContext ptr -> mjrRect

(** Draw rectangle. *)
val mjr_rectangle : mjrRect -> float -> float -> float -> float -> unit

(** Draw rectangle with centered text. *)
val mjr_label
  :  mjrRect
  -> int
  -> string
  -> float
  -> float
  -> float
  -> float
  -> float
  -> float
  -> float
  -> mjrContext ptr
  -> unit

(** Draw 2D figure. *)
val mjr_figure : mjrRect -> mjvFigure ptr -> mjrContext ptr -> unit

(** Render 3D scene. *)
val mjr_render : mjrRect -> mjvScene ptr -> mjrContext ptr -> unit

(** Call glFinish. *)
val mjr_finish : unit -> unit

(** Call glGetError and return result. *)
val mjr_getError : unit -> int

(** Find first rectangle containing mouse, -1: not found. *)
val mjr_findRect : int -> int -> int -> mjrRect ptr -> int

(** Main error function; does not return to caller. *)
val mju_error : string -> unit

(** Error function with int argument; msg is a printf format string. *)
val mju_error_i : string -> int -> unit

(** Error function with string argument. *)
val mju_error_s : string -> string -> unit

(** Main warning function; returns to caller. *)
val mju_warning : string -> unit

(** Warning function with int argument. *)
val mju_warning_i : string -> int -> unit

(** Warning function with string argument. *)
val mju_warning_s : string -> string -> unit

(** Clear user error and memory handlers. *)
val mju_clearHandlers : unit -> unit

(** Allocate memory; byte-align on 8; pad size to multiple of 8. *)
val mju_malloc : Unsigned.Size_t.t -> unit ptr

(** Free memory, using free() by default. *)
val mju_free : unit ptr -> unit

(** High-level warning function: count warnings in mjData, print only the first. *)
val mj_warning : mjData ptr -> int -> int -> unit

(** Write [datetime, type: message] to MUJOCO_LOG.TXT. *)
val mju_writeLog : string -> string -> unit

(** Set res = 0. *)
val mju_zero3 : float ptr -> unit

(** Set res = vec. *)
val mju_copy3 : float ptr -> float ptr -> unit

(** Set res = vec*scl. *)
val mju_scl3 : float ptr -> float ptr -> float -> unit

(** Set res = vec1 + vec2. *)
val mju_add3 : float ptr -> float ptr -> float ptr -> unit

(** Set res = vec1 - vec2. *)
val mju_sub3 : float ptr -> float ptr -> float ptr -> unit

(** Set res = res + vec. *)
val mju_addTo3 : float ptr -> float ptr -> unit

(** Set res = res - vec. *)
val mju_subFrom3 : float ptr -> float ptr -> unit

(** Set res = res + vec*scl. *)
val mju_addToScl3 : float ptr -> float ptr -> float -> unit

(** Set res = vec1 + vec2*scl. *)
val mju_addScl3 : float ptr -> float ptr -> float ptr -> float -> unit

(** Normalize vector, return length before normalization. *)
val mju_normalize3 : float ptr -> float

(** Return vector length (without normalizing the vector). *)
val mju_norm3 : float ptr -> float

(** Return dot-product of vec1 and vec2. *)
val mju_dot3 : float ptr -> float ptr -> float

(** Return Cartesian distance between 3D vectors pos1 and pos2. *)
val mju_dist3 : float ptr -> float ptr -> float

(** Multiply vector by 3D rotation matrix: res = mat * vec. *)
val mju_rotVecMat : float ptr -> float ptr -> float ptr -> unit

(** Multiply vector by transposed 3D rotation matrix: res = mat' * vec. *)
val mju_rotVecMatT : float ptr -> float ptr -> float ptr -> unit

(** Compute cross-product: res = cross(a, b). *)
val mju_cross : float ptr -> float ptr -> float ptr -> unit

(** Set res = 0. *)
val mju_zero4 : float ptr -> unit

(** Set res = (1,0,0,0). *)
val mju_unit4 : float ptr -> unit

(** Set res = vec. *)
val mju_copy4 : float ptr -> float ptr -> unit

(** Normalize vector, return length before normalization. *)
val mju_normalize4 : float ptr -> float

(** Set res = 0. *)
val mju_zero : float ptr -> int -> unit

(** Set res = vec. *)
val mju_copy : float ptr -> float ptr -> int -> unit

(** Return sum(vec). *)
val mju_sum : float ptr -> int -> float

(** Return L1 norm: sum(abs(vec)). *)
val mju_L1 : float ptr -> int -> float

(** Set res = vec*scl. *)
val mju_scl : float ptr -> float ptr -> float -> int -> unit

(** Set res = vec1 + vec2. *)
val mju_add : float ptr -> float ptr -> float ptr -> int -> unit

(** Set res = vec1 - vec2. *)
val mju_sub : float ptr -> float ptr -> float ptr -> int -> unit

(** Set res = res + vec. *)
val mju_addTo : float ptr -> float ptr -> int -> unit

(** Set res = res - vec. *)
val mju_subFrom : float ptr -> float ptr -> int -> unit

(** Set res = res + vec*scl. *)
val mju_addToScl : float ptr -> float ptr -> float -> int -> unit

(** Set res = vec1 + vec2*scl. *)
val mju_addScl : float ptr -> float ptr -> float ptr -> float -> int -> unit

(** Normalize vector, return length before normalization. *)
val mju_normalize : float ptr -> int -> float

(** Return vector length (without normalizing vector). *)
val mju_norm : float ptr -> int -> float

(** Return dot-product of vec1 and vec2. *)
val mju_dot : float ptr -> float ptr -> int -> float

(** Multiply matrix and vector: res = mat * vec. *)
val mju_mulMatVec : float ptr -> float ptr -> float ptr -> int -> int -> unit

(** Multiply transposed matrix and vector: res = mat' * vec. *)
val mju_mulMatTVec : float ptr -> float ptr -> float ptr -> int -> int -> unit

(** Transpose matrix: res = mat'. *)
val mju_transpose : float ptr -> float ptr -> int -> int -> unit

(** Multiply matrices: res = mat1 * mat2. *)
val mju_mulMatMat : float ptr -> float ptr -> float ptr -> int -> int -> int -> unit

(** Multiply matrices, second argument transposed: res = mat1 * mat2'. *)
val mju_mulMatMatT : float ptr -> float ptr -> float ptr -> int -> int -> int -> unit

(** Multiply matrices, first argument transposed: res = mat1' * mat2. *)
val mju_mulMatTMat : float ptr -> float ptr -> float ptr -> int -> int -> int -> unit

(** Set res = mat' * diag * mat if diag is not NULL, and res = mat' * mat otherwise. *)
val mju_sqrMatTD : float ptr -> float ptr -> float ptr -> int -> int -> unit

(** Coordinate transform of 6D motion or force vector in rotation:translation format.
 rotnew2old is 3-by-3, NULL means no rotation; flg_force specifies force or motion type. *)
val mju_transformSpatial
  :  float ptr
  -> float ptr
  -> int
  -> float ptr
  -> float ptr
  -> float ptr
  -> unit

(** Rotate vector by quaternion. *)
val mju_rotVecQuat : float ptr -> float ptr -> float ptr -> unit

(** Conjugate quaternion, corresponding to opposite rotation. *)
val mju_negQuat : float ptr -> float ptr -> unit

(** Muiltiply quaternions. *)
val mju_mulQuat : float ptr -> float ptr -> float ptr -> unit

(** Muiltiply quaternion and axis. *)
val mju_mulQuatAxis : float ptr -> float ptr -> float ptr -> unit

(** Convert axisAngle to quaternion. *)
val mju_axisAngle2Quat : float ptr -> float ptr -> float -> unit

(** Convert quaternion (corresponding to orientation difference) to 3D velocity. *)
val mju_quat2Vel : float ptr -> float ptr -> float -> unit

(** Subtract quaternions, express as 3D velocity: qb*quat(res) = qa. *)
val mju_subQuat : float ptr -> float ptr -> float ptr -> unit

(** Convert quaternion to 3D rotation matrix. *)
val mju_quat2Mat : float ptr -> float ptr -> unit

(** Convert 3D rotation matrix to quaterion. *)
val mju_mat2Quat : float ptr -> float ptr -> unit

(** Compute time-derivative of quaternion, given 3D rotational velocity. *)
val mju_derivQuat : float ptr -> float ptr -> float ptr -> unit

(** Integrate quaterion given 3D angular velocity. *)
val mju_quatIntegrate : float ptr -> float ptr -> float -> unit

(** Construct quaternion performing rotation from z-axis to given vector. *)
val mju_quatZ2Vec : float ptr -> float ptr -> unit

(** Multiply two poses. *)
val mju_mulPose
  :  float ptr
  -> float ptr
  -> float ptr
  -> float ptr
  -> float ptr
  -> float ptr
  -> unit

(** Conjugate pose, corresponding to the opposite spatial transformation. *)
val mju_negPose : float ptr -> float ptr -> float ptr -> float ptr -> unit

(** Transform vector by pose. *)
val mju_trnVecPose : float ptr -> float ptr -> float ptr -> float ptr -> unit

(** Cholesky decomposition: mat = L*L'; return rank. *)
val mju_cholFactor : float ptr -> int -> float -> int

(** Solve mat * res = vec, where mat is Cholesky-factorized *)
val mju_cholSolve : float ptr -> float ptr -> float ptr -> int -> unit

(** Cholesky rank-one update: L*L' +/- x*x'; return rank. *)
val mju_cholUpdate : float ptr -> float ptr -> int -> int -> int

(** Eigenvalue decomposition of symmetric 3x3 matrix. *)
val mju_eig3 : float ptr -> float ptr -> float ptr -> float ptr -> int

(** Muscle active force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax). *)
val mju_muscleGain : float -> float -> float ptr -> float -> float ptr -> float

(** Muscle passive force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax). *)
val mju_muscleBias : float -> float ptr -> float -> float ptr -> float

(** Muscle activation dynamics, prm = (tau_act, tau_deact). *)
val mju_muscleDynamics : float -> float -> float ptr -> float

(** Convert contact force to pyramid representation. *)
val mju_encodePyramid : float ptr -> float ptr -> float ptr -> int -> unit

(** Convert pyramid representation to contact force. *)
val mju_decodePyramid : float ptr -> float ptr -> float ptr -> int -> unit

(** Integrate spring-damper analytically, return pos(dt). *)
val mju_springDamper : float -> float -> float -> float -> float -> float

(** Return min(a,b) with single evaluation of a and b. *)
val mju_min : float -> float -> float

(** Return max(a,b) with single evaluation of a and b. *)
val mju_max : float -> float -> float

(** Return sign of x: +1, -1 or 0. *)
val mju_sign : float -> float

(** Round x to nearest integer. *)
val mju_round : float -> int

(** Convert type name to type id (mjtObj). *)
val mju_str2Type : string -> int

(** Return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise. Used by check functions. *)
val mju_isBad : float -> int

(** Return 1 if all elements are 0. *)
val mju_isZero : float ptr -> int -> int

(** Standard normal random number generator (optional second number). *)
val mju_standardNormal : float ptr -> float

(** Convert from float to mjtNum. *)
val mju_f2n : float ptr -> float ptr -> int -> unit

(** Convert from mjtNum to float. *)
val mju_n2f : float ptr -> float ptr -> int -> unit

(** Convert from double to mjtNum. *)
val mju_d2n : float ptr -> float ptr -> int -> unit

(** Convert from mjtNum to double. *)
val mju_n2d : float ptr -> float ptr -> int -> unit

(** Insertion sort, resulting list is in increasing order. *)
val mju_insertionSort : float ptr -> int -> unit

(** Integer insertion sort, resulting list is in increasing order. *)
val mju_insertionSortInt : int ptr -> int -> unit

(** Generate Halton sequence. *)
val mju_Halton : int -> int -> float

(** Call strncpy, then set dst[n-1] = 0. *)
val mju_strncpy : char -> char -> int -> string

(** Sigmoid function over 0<=x<=1 constructed from half-quadratics. *)
val mju_sigmoid : float -> float
