(* THIS FILE IS GENERATED AUTOMATICALLY, DO NOT EDIT BY HAND *)

open Ctypes
module Typs = Typs
open Typs

module Bindings (F : FOREIGN) = struct
  open F

  (** ---------------------- Activation ----------------------------------------------------- *)

  (** Return 1 (for backward compatibility). *)
  let mj_activate = foreign "mj_activate" (string @-> returning int)

  (** Do nothing (for backward compatibility). *)
  let mj_deactivate = foreign "mj_deactivate" (void @-> returning void)

  (** ---------------------- Virtual file system -------------------------------------------- *)

  (** Initialize VFS to empty (no deallocation). *)
  let mj_defaultVFS = foreign "mj_defaultVFS" (ptr mjVFS @-> returning void)

  (** Add file to VFS, return 0: success, 1: full, 2: repeated name, -1: not found on disk. *)
  let mj_addFileVFS =
    foreign "mj_addFileVFS" (ptr mjVFS @-> string @-> string @-> returning int)


  (** Make empty file in VFS, return 0: success, 1: full, 2: repeated name. *)
  let mj_makeEmptyFileVFS =
    foreign "mj_makeEmptyFileVFS" (ptr mjVFS @-> string @-> int @-> returning int)


  (** Return file index in VFS, or -1 if not found in VFS. *)
  let mj_findFileVFS = foreign "mj_findFileVFS" (ptr mjVFS @-> string @-> returning int)

  (** Delete file from VFS, return 0: success, -1: not found in VFS. *)
  let mj_deleteFileVFS =
    foreign "mj_deleteFileVFS" (ptr mjVFS @-> string @-> returning int)


  (** Delete all files from VFS. *)
  let mj_deleteVFS = foreign "mj_deleteVFS" (ptr mjVFS @-> returning void)

  (** ---------------------- Parse and compile ---------------------------------------------- *)

  (** Parse XML file in MJCF or URDF format, compile it, return low-level model.
 If vfs is not NULL, look up files in vfs before reading from disk.
 If error is not NULL, it must have size error_sz. *)
  let mj_loadXML =
    foreign
      "mj_loadXML"
      (string @-> ptr mjVFS @-> string @-> int @-> returning (ptr mjModel))


  (** Update XML data structures with info from low-level model, save as MJCF.
 If error is not NULL, it must have size error_sz. *)
  let mj_saveLastXML =
    foreign "mj_saveLastXML" (string @-> ptr mjModel @-> string @-> int @-> returning int)


  (** Free last XML model if loaded. Called internally at each load. *)
  let mj_freeLastXML = foreign "mj_freeLastXML" (void @-> returning void)

  (** Print internal XML schema as plain text or HTML, with style-padding or &nbsp;. *)
  let mj_printSchema =
    foreign "mj_printSchema" (string @-> string @-> int @-> int @-> int @-> returning int)


  (** ---------------------- Main simulation ------------------------------------------------ *)

  (** Advance simulation, use control callback to obtain external force and control. *)
  let mj_step = foreign "mj_step" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Advance simulation in two steps: before external force and control is set by user. *)
  let mj_step1 = foreign "mj_step1" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Advance simulation in two steps: after external force and control is set by user. *)
  let mj_step2 = foreign "mj_step2" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Forward dynamics: same as mj_step but do not integrate in time. *)
  let mj_forward = foreign "mj_forward" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Inverse dynamics: qacc must be set before calling. *)
  let mj_inverse = foreign "mj_inverse" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Forward dynamics with skip; skipstage is mjtStage. *)
  let mj_forwardSkip =
    foreign
      "mj_forwardSkip"
      (ptr mjModel @-> ptr mjData @-> int @-> int @-> returning void)


  (** Inverse dynamics with skip; skipstage is mjtStage. *)
  let mj_inverseSkip =
    foreign
      "mj_inverseSkip"
      (ptr mjModel @-> ptr mjData @-> int @-> int @-> returning void)


  (** ---------------------- Initialization ------------------------------------------------- *)

  (** Set default options for length range computation. *)
  let mj_defaultLROpt = foreign "mj_defaultLROpt" (ptr mjLROpt @-> returning void)

  (** Set solver parameters to default values. *)
  let mj_defaultSolRefImp =
    foreign "mj_defaultSolRefImp" (ptr mjtNum @-> ptr mjtNum @-> returning void)


  (** Set physics options to default values. *)
  let mj_defaultOption = foreign "mj_defaultOption" (ptr mjOption @-> returning void)

  (** Set visual options to default values. *)
  let mj_defaultVisual = foreign "mj_defaultVisual" (ptr mjVisual @-> returning void)

  (** Copy mjModel, allocate new if dest is NULL. *)
  let mj_copyModel =
    foreign "mj_copyModel" (ptr mjModel @-> ptr mjModel @-> returning (ptr mjModel))


  (** Save model to binary MJB file or memory buffer; buffer has precedence when given. *)
  let mj_saveModel =
    foreign "mj_saveModel" (ptr mjModel @-> string @-> ptr void @-> int @-> returning void)


  (** Load model from binary MJB file.
 If vfs is not NULL, look up file in vfs before reading from disk. *)
  let mj_loadModel =
    foreign "mj_loadModel" (string @-> ptr mjVFS @-> returning (ptr mjModel))


  (** Free memory allocation in model. *)
  let mj_deleteModel = foreign "mj_deleteModel" (ptr mjModel @-> returning void)

  (** Return size of buffer needed to hold model. *)
  let mj_sizeModel = foreign "mj_sizeModel" (ptr mjModel @-> returning int)

  (** Allocate mjData correponding to given model. *)
  let mj_makeData = foreign "mj_makeData" (ptr mjModel @-> returning (ptr mjData))

  (** Copy mjData. *)
  let mj_copyData =
    foreign
      "mj_copyData"
      (ptr mjData @-> ptr mjModel @-> ptr mjData @-> returning (ptr mjData))


  (** Reset data to defaults. *)
  let mj_resetData = foreign "mj_resetData" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Reset data to defaults, fill everything else with debug_value. *)
  let mj_resetDataDebug =
    foreign "mj_resetDataDebug" (ptr mjModel @-> ptr mjData @-> uchar @-> returning void)


  (** Reset data, set fields from specified keyframe. *)
  let mj_resetDataKeyframe =
    foreign "mj_resetDataKeyframe" (ptr mjModel @-> ptr mjData @-> int @-> returning void)


  (** Allocate array of specified size on mjData stack. Call mju_error on stack overflow. *)
  let mj_stackAlloc =
    foreign "mj_stackAlloc" (ptr mjData @-> int @-> returning (ptr mjtNum))


  (** Free memory allocation in mjData. *)
  let mj_deleteData = foreign "mj_deleteData" (ptr mjData @-> returning void)

  (** Reset all callbacks to NULL pointers (NULL is the default). *)
  let mj_resetCallbacks = foreign "mj_resetCallbacks" (void @-> returning void)

  (** Set constant fields of mjModel, corresponding to qpos0 configuration. *)
  let mj_setConst = foreign "mj_setConst" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Set actuator_lengthrange for specified actuator; return 1 if ok, 0 if error. *)
  let mj_setLengthRange =
    foreign
      "mj_setLengthRange"
      (ptr mjModel
      @-> ptr mjData
      @-> int
      @-> ptr mjLROpt
      @-> string
      @-> int
      @-> returning int)


  (** ---------------------- Printing ------------------------------------------------------- *)

  (** Print model to text file. *)
  let mj_printModel = foreign "mj_printModel" (ptr mjModel @-> string @-> returning void)

  (** Print data to text file. *)
  let mj_printData =
    foreign "mj_printData" (ptr mjModel @-> ptr mjData @-> string @-> returning void)


  (** Print matrix to screen. *)
  let mju_printMat = foreign "mju_printMat" (ptr mjtNum @-> int @-> int @-> returning void)

  (** Print sparse matrix to screen. *)
  let mju_printMatSparse =
    foreign
      "mju_printMatSparse"
      (ptr mjtNum @-> int @-> ptr int @-> ptr int @-> ptr int @-> returning void)


  (** ---------------------- Components ----------------------------------------------------- *)

  (** Run position-dependent computations. *)
  let mj_fwdPosition =
    foreign "mj_fwdPosition" (ptr mjModel @-> ptr mjData @-> returning void)


  (** Run velocity-dependent computations. *)
  let mj_fwdVelocity =
    foreign "mj_fwdVelocity" (ptr mjModel @-> ptr mjData @-> returning void)


  (** Compute actuator force qfrc_actuation. *)
  let mj_fwdActuation =
    foreign "mj_fwdActuation" (ptr mjModel @-> ptr mjData @-> returning void)


  (** Add up all non-constraint forces, compute qacc_unc. *)
  let mj_fwdAcceleration =
    foreign "mj_fwdAcceleration" (ptr mjModel @-> ptr mjData @-> returning void)


  (** Run selected constraint solver. *)
  let mj_fwdConstraint =
    foreign "mj_fwdConstraint" (ptr mjModel @-> ptr mjData @-> returning void)


  (** Euler integrator, semi-implicit in velocity. *)
  let mj_Euler = foreign "mj_Euler" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Runge-Kutta explicit order-N integrator. *)
  let mj_RungeKutta =
    foreign "mj_RungeKutta" (ptr mjModel @-> ptr mjData @-> int @-> returning void)


  (** Run position-dependent computations in inverse dynamics. *)
  let mj_invPosition =
    foreign "mj_invPosition" (ptr mjModel @-> ptr mjData @-> returning void)


  (** Run velocity-dependent computations in inverse dynamics. *)
  let mj_invVelocity =
    foreign "mj_invVelocity" (ptr mjModel @-> ptr mjData @-> returning void)


  (** Apply the analytical formula for inverse constraint dynamics. *)
  let mj_invConstraint =
    foreign "mj_invConstraint" (ptr mjModel @-> ptr mjData @-> returning void)


  (** Compare forward and inverse dynamics, save results in fwdinv. *)
  let mj_compareFwdInv =
    foreign "mj_compareFwdInv" (ptr mjModel @-> ptr mjData @-> returning void)


  (** ---------------------- Sub components ------------------------------------------------- *)

  (** Evaluate position-dependent sensors. *)
  let mj_sensorPos = foreign "mj_sensorPos" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Evaluate velocity-dependent sensors. *)
  let mj_sensorVel = foreign "mj_sensorVel" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Evaluate acceleration and force-dependent sensors. *)
  let mj_sensorAcc = foreign "mj_sensorAcc" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Evaluate position-dependent energy (potential). *)
  let mj_energyPos = foreign "mj_energyPos" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Evaluate velocity-dependent energy (kinetic). *)
  let mj_energyVel = foreign "mj_energyVel" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Check qpos, reset if any element is too big or nan. *)
  let mj_checkPos = foreign "mj_checkPos" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Check qvel, reset if any element is too big or nan. *)
  let mj_checkVel = foreign "mj_checkVel" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Check qacc, reset if any element is too big or nan. *)
  let mj_checkAcc = foreign "mj_checkAcc" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Run forward kinematics. *)
  let mj_kinematics =
    foreign "mj_kinematics" (ptr mjModel @-> ptr mjData @-> returning void)


  (** Map inertias and motion dofs to global frame centered at CoM. *)
  let mj_comPos = foreign "mj_comPos" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Compute camera and light positions and orientations. *)
  let mj_camlight = foreign "mj_camlight" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Compute tendon lengths, velocities and moment arms. *)
  let mj_tendon = foreign "mj_tendon" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Compute actuator transmission lengths and moments. *)
  let mj_transmission =
    foreign "mj_transmission" (ptr mjModel @-> ptr mjData @-> returning void)


  (** Run composite rigid body inertia algorithm (CRB). *)
  let mj_crb = foreign "mj_crb" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Compute sparse L'*D*L factorizaton of inertia matrix. *)
  let mj_factorM = foreign "mj_factorM" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Solve linear system M * x = y using factorization:  x = inv(L'*D*L)*y *)
  let mj_solveM =
    foreign
      "mj_solveM"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  (** Half of linear solve:  x = sqrt(inv(D))*inv(L')*y *)
  let mj_solveM2 =
    foreign
      "mj_solveM2"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  (** Compute cvel, cdof_dot. *)
  let mj_comVel = foreign "mj_comVel" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Compute qfrc_passive from spring-dampers, viscosity and density. *)
  let mj_passive = foreign "mj_passive" (ptr mjModel @-> ptr mjData @-> returning void)

  (** subtree linear velocity and angular momentum *)
  let mj_subtreeVel =
    foreign "mj_subtreeVel" (ptr mjModel @-> ptr mjData @-> returning void)


  (** RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term. *)
  let mj_rne =
    foreign "mj_rne" (ptr mjModel @-> ptr mjData @-> int @-> ptr mjtNum @-> returning void)


  (** RNE with complete data: compute cacc, cfrc_ext, cfrc_int. *)
  let mj_rnePostConstraint =
    foreign "mj_rnePostConstraint" (ptr mjModel @-> ptr mjData @-> returning void)


  (** Run collision detection. *)
  let mj_collision = foreign "mj_collision" (ptr mjModel @-> ptr mjData @-> returning void)

  (** Construct constraints. *)
  let mj_makeConstraint =
    foreign "mj_makeConstraint" (ptr mjModel @-> ptr mjData @-> returning void)


  (** Compute inverse constaint inertia efc_AR. *)
  let mj_projectConstraint =
    foreign "mj_projectConstraint" (ptr mjModel @-> ptr mjData @-> returning void)


  (** Compute efc_vel, efc_aref. *)
  let mj_referenceConstraint =
    foreign "mj_referenceConstraint" (ptr mjModel @-> ptr mjData @-> returning void)


  (** Compute efc_state, efc_force, qfrc_constraint, and (optionally) cone Hessians.
 If cost is not NULL, set *cost = s(jar) where jar = Jac*qacc-aref. *)
  let mj_constraintUpdate =
    foreign
      "mj_constraintUpdate"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  (** ---------------------- Support -------------------------------------------------------- *)

  (** Add contact to d->contact list; return 0 if success; 1 if buffer full. *)
  let mj_addContact =
    foreign
      "mj_addContact"
      (ptr mjModel @-> ptr mjData @-> ptr mjContact @-> returning int)


  (** Determine type of friction cone. *)
  let mj_isPyramidal = foreign "mj_isPyramidal" (ptr mjModel @-> returning int)

  (** Determine type of constraint Jacobian. *)
  let mj_isSparse = foreign "mj_isSparse" (ptr mjModel @-> returning int)

  (** Determine type of solver (PGS is dual, CG and Newton are primal). *)
  let mj_isDual = foreign "mj_isDual" (ptr mjModel @-> returning int)

  (** Multiply dense or sparse constraint Jacobian by vector. *)
  let mj_mulJacVec =
    foreign
      "mj_mulJacVec"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  (** Multiply dense or sparse constraint Jacobian transpose by vector. *)
  let mj_mulJacTVec =
    foreign
      "mj_mulJacTVec"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  (** Compute 3/6-by-nv end-effector Jacobian of global point attached to given body. *)
  let mj_jac =
    foreign
      "mj_jac"
      (ptr mjModel
      @-> ptr mjData
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> int
      @-> returning void)


  (** Compute body frame end-effector Jacobian. *)
  let mj_jacBody =
    foreign
      "mj_jacBody"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  (** Compute body center-of-mass end-effector Jacobian. *)
  let mj_jacBodyCom =
    foreign
      "mj_jacBodyCom"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  (** Compute geom end-effector Jacobian. *)
  let mj_jacGeom =
    foreign
      "mj_jacGeom"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  (** Compute site end-effector Jacobian. *)
  let mj_jacSite =
    foreign
      "mj_jacSite"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  (** Compute translation end-effector Jacobian of point, and rotation Jacobian of axis. *)
  let mj_jacPointAxis =
    foreign
      "mj_jacPointAxis"
      (ptr mjModel
      @-> ptr mjData
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> int
      @-> returning void)


  (** Get id of object with specified name, return -1 if not found; type is mjtObj. *)
  let mj_name2id = foreign "mj_name2id" (ptr mjModel @-> int @-> string @-> returning int)

  (** Convert sparse inertia matrix M into full (i.e. dense) matrix. *)
  let mj_fullM =
    foreign "mj_fullM" (ptr mjModel @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  (** Multiply vector by inertia matrix. *)
  let mj_mulM =
    foreign
      "mj_mulM"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  (** Multiply vector by (inertia matrix)^(1/2). *)
  let mj_mulM2 =
    foreign
      "mj_mulM2"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  (** Add inertia matrix to destination matrix.
 Destination can be sparse uncompressed, or dense when all int* are NULL *)
  let mj_addM =
    foreign
      "mj_addM"
      (ptr mjModel
      @-> ptr mjData
      @-> ptr mjtNum
      @-> ptr int
      @-> ptr int
      @-> ptr int
      @-> returning void)


  (** Apply cartesian force and torque (outside xfrc_applied mechanism). *)
  let mj_applyFT =
    foreign
      "mj_applyFT"
      (ptr mjModel
      @-> ptr mjData
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> int
      @-> ptr mjtNum
      @-> returning void)


  (** Compute object 6D velocity in object-centered frame, world/local orientation. *)
  let mj_objectVelocity =
    foreign
      "mj_objectVelocity"
      (ptr mjModel
      @-> ptr mjData
      @-> int
      @-> int
      @-> ptr mjtNum
      @-> int
      @-> returning void)


  (** Compute object 6D acceleration in object-centered frame, world/local orientation. *)
  let mj_objectAcceleration =
    foreign
      "mj_objectAcceleration"
      (ptr mjModel
      @-> ptr mjData
      @-> int
      @-> int
      @-> ptr mjtNum
      @-> int
      @-> returning void)


  (** Extract 6D force:torque for one contact, in contact frame. *)
  let mj_contactForce =
    foreign
      "mj_contactForce"
      (ptr mjModel @-> ptr mjData @-> int @-> ptr mjtNum @-> returning void)


  (** Compute velocity by finite-differencing two positions. *)
  let mj_differentiatePos =
    foreign
      "mj_differentiatePos"
      (ptr mjModel
      @-> ptr mjtNum
      @-> mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> returning void)


  (** Integrate position with given velocity. *)
  let mj_integratePos =
    foreign
      "mj_integratePos"
      (ptr mjModel @-> ptr mjtNum @-> ptr mjtNum @-> mjtNum @-> returning void)


  (** Normalize all quaterions in qpos-type vector. *)
  let mj_normalizeQuat =
    foreign "mj_normalizeQuat" (ptr mjModel @-> ptr mjtNum @-> returning void)


  (** Map from body local to global Cartesian coordinates. *)
  let mj_local2Global =
    foreign
      "mj_local2Global"
      (ptr mjData
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> int
      @-> mjtByte
      @-> returning void)


  (** Sum all body masses. *)
  let mj_getTotalmass = foreign "mj_getTotalmass" (ptr mjModel @-> returning mjtNum)

  (** Scale body masses and inertias to achieve specified total mass. *)
  let mj_setTotalmass =
    foreign "mj_setTotalmass" (ptr mjModel @-> mjtNum @-> returning void)


  (** Return version number: 1.0.2 is encoded as 102. *)
  let mj_version = foreign "mj_version" (void @-> returning int)

  (** ---------------------- Ray collisions ------------------------------------------------- *)

  (** Intersect ray (pnt+x*vec, x>=0) with visible geoms, except geoms in bodyexclude.
 Return geomid and distance (x) to nearest surface, or -1 if no intersection.
 geomgroup, flg_static are as in mjvOption; geomgroup==NULL skips group exclusion. *)
  let mj_ray =
    foreign
      "mj_ray"
      (ptr mjModel
      @-> ptr mjData
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtByte
      @-> mjtByte
      @-> int
      @-> ptr int
      @-> returning mjtNum)


  (** Interect ray with hfield, return nearest distance or -1 if no intersection. *)
  let mj_rayHfield =
    foreign
      "mj_rayHfield"
      (ptr mjModel
      @-> ptr mjData
      @-> int
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> returning mjtNum)


  (** Interect ray with mesh, return nearest distance or -1 if no intersection. *)
  let mj_rayMesh =
    foreign
      "mj_rayMesh"
      (ptr mjModel
      @-> ptr mjData
      @-> int
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> returning mjtNum)


  (** Interect ray with pure geom, return nearest distance or -1 if no intersection. *)
  let mju_rayGeom =
    foreign
      "mju_rayGeom"
      (ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> int
      @-> returning mjtNum)


  (** Interect ray with skin, return nearest vertex id. *)
  let mju_raySkin =
    foreign
      "mju_raySkin"
      (int
      @-> int
      @-> ptr int
      @-> ptr float
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr int
      @-> returning mjtNum)


  (** ---------------------- Interaction ---------------------------------------------------- *)

  (** Set default camera. *)
  let mjv_defaultCamera = foreign "mjv_defaultCamera" (ptr mjvCamera @-> returning void)

  (** Set default perturbation. *)
  let mjv_defaultPerturb = foreign "mjv_defaultPerturb" (ptr mjvPerturb @-> returning void)

  (** Transform pose from room to model space. *)
  let mjv_room2model =
    foreign
      "mjv_room2model"
      (ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjvScene
      @-> returning void)


  (** Transform pose from model to room space. *)
  let mjv_model2room =
    foreign
      "mjv_model2room"
      (ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjvScene
      @-> returning void)


  (** Get camera info in model space; average left and right OpenGL cameras. *)
  let mjv_cameraInModel =
    foreign
      "mjv_cameraInModel"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> ptr mjvScene @-> returning void)


  (** Get camera info in room space; average left and right OpenGL cameras. *)
  let mjv_cameraInRoom =
    foreign
      "mjv_cameraInRoom"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> ptr mjvScene @-> returning void)


  (** Get frustum height at unit distance from camera; average left and right OpenGL cameras. *)
  let mjv_frustumHeight = foreign "mjv_frustumHeight" (ptr mjvScene @-> returning mjtNum)

  (** Rotate 3D vec in horizontal plane by angle between (0,1) and (forward_x,forward_y). *)
  let mjv_alignToCamera =
    foreign
      "mjv_alignToCamera"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  (** Move camera with mouse; action is mjtMouse. *)
  let mjv_moveCamera =
    foreign
      "mjv_moveCamera"
      (ptr mjModel
      @-> int
      @-> mjtNum
      @-> mjtNum
      @-> ptr mjvScene
      @-> ptr mjvCamera
      @-> returning void)


  (** Move perturb object with mouse; action is mjtMouse. *)
  let mjv_movePerturb =
    foreign
      "mjv_movePerturb"
      (ptr mjModel
      @-> ptr mjData
      @-> int
      @-> mjtNum
      @-> mjtNum
      @-> ptr mjvScene
      @-> ptr mjvPerturb
      @-> returning void)


  (** Move model with mouse; action is mjtMouse. *)
  let mjv_moveModel =
    foreign
      "mjv_moveModel"
      (ptr mjModel
      @-> int
      @-> mjtNum
      @-> mjtNum
      @-> ptr mjtNum
      @-> ptr mjvScene
      @-> returning void)


  (** Copy perturb pos,quat from selected body; set scale for perturbation. *)
  let mjv_initPerturb =
    foreign
      "mjv_initPerturb"
      (ptr mjModel @-> ptr mjData @-> ptr mjvScene @-> ptr mjvPerturb @-> returning void)


  (** Set perturb pos,quat in d->mocap when selected body is mocap, and in d->qpos otherwise.
 Write d->qpos only if flg_paused and subtree root for selected body has free joint. *)
  let mjv_applyPerturbPose =
    foreign
      "mjv_applyPerturbPose"
      (ptr mjModel @-> ptr mjData @-> ptr mjvPerturb @-> int @-> returning void)


  (** Set perturb force,torque in d->xfrc_applied, if selected body is dynamic. *)
  let mjv_applyPerturbForce =
    foreign
      "mjv_applyPerturbForce"
      (ptr mjModel @-> ptr mjData @-> ptr mjvPerturb @-> returning void)


  (** Return the average of two OpenGL cameras. *)
  let mjv_averageCamera =
    foreign
      "mjv_averageCamera"
      (ptr mjvGLCamera @-> ptr mjvGLCamera @-> returning mjvGLCamera)


  (** Select geom or skin with mouse, return bodyid; -1: none selected. *)
  let mjv_select =
    foreign
      "mjv_select"
      (ptr mjModel
      @-> ptr mjData
      @-> ptr mjvOption
      @-> mjtNum
      @-> mjtNum
      @-> mjtNum
      @-> ptr mjvScene
      @-> ptr mjtNum
      @-> ptr int
      @-> ptr int
      @-> returning int)


  (** ---------------------- Visualization -------------------------------------------------- *)

  (** Set default visualization options. *)
  let mjv_defaultOption = foreign "mjv_defaultOption" (ptr mjvOption @-> returning void)

  (** Set default figure. *)
  let mjv_defaultFigure = foreign "mjv_defaultFigure" (ptr mjvFigure @-> returning void)

  (** Initialize given geom fields when not NULL, set the rest to their default values. *)
  let mjv_initGeom =
    foreign
      "mjv_initGeom"
      (ptr mjvGeom
      @-> int
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr float
      @-> returning void)


  (** Set (type, size, pos, mat) for connector-type geom between given points.
 Assume that mjv_initGeom was already called to set all other properties. *)
  let mjv_makeConnector =
    foreign
      "mjv_makeConnector"
      (ptr mjvGeom
      @-> int
      @-> mjtNum
      @-> mjtNum
      @-> mjtNum
      @-> mjtNum
      @-> mjtNum
      @-> mjtNum
      @-> mjtNum
      @-> returning void)


  (** Set default abstract scene. *)
  let mjv_defaultScene = foreign "mjv_defaultScene" (ptr mjvScene @-> returning void)

  (** Allocate resources in abstract scene. *)
  let mjv_makeScene =
    foreign "mjv_makeScene" (ptr mjModel @-> ptr mjvScene @-> int @-> returning void)


  (** Free abstract scene. *)
  let mjv_freeScene = foreign "mjv_freeScene" (ptr mjvScene @-> returning void)

  (** Update entire scene given model state. *)
  let mjv_updateScene =
    foreign
      "mjv_updateScene"
      (ptr mjModel
      @-> ptr mjData
      @-> ptr mjvOption
      @-> ptr mjvPerturb
      @-> ptr mjvCamera
      @-> int
      @-> ptr mjvScene
      @-> returning void)


  (** Add geoms from selected categories. *)
  let mjv_addGeoms =
    foreign
      "mjv_addGeoms"
      (ptr mjModel
      @-> ptr mjData
      @-> ptr mjvOption
      @-> ptr mjvPerturb
      @-> int
      @-> ptr mjvScene
      @-> returning void)


  (** Make list of lights. *)
  let mjv_makeLights =
    foreign
      "mjv_makeLights"
      (ptr mjModel @-> ptr mjData @-> ptr mjvScene @-> returning void)


  (** Update camera. *)
  let mjv_updateCamera =
    foreign
      "mjv_updateCamera"
      (ptr mjModel @-> ptr mjData @-> ptr mjvCamera @-> ptr mjvScene @-> returning void)


  (** Update skins. *)
  let mjv_updateSkin =
    foreign
      "mjv_updateSkin"
      (ptr mjModel @-> ptr mjData @-> ptr mjvScene @-> returning void)


  (** ---------------------- OpenGL rendering ----------------------------------------------- *)

  (** Set default mjrContext. *)
  let mjr_defaultContext = foreign "mjr_defaultContext" (ptr mjrContext @-> returning void)

  (** Allocate resources in custom OpenGL context; fontscale is mjtFontScale. *)
  let mjr_makeContext =
    foreign "mjr_makeContext" (ptr mjModel @-> ptr mjrContext @-> int @-> returning void)


  (** Change font of existing context. *)
  let mjr_changeFont = foreign "mjr_changeFont" (int @-> ptr mjrContext @-> returning void)

  (** Add Aux buffer with given index to context; free previous Aux buffer. *)
  let mjr_addAux =
    foreign
      "mjr_addAux"
      (int @-> int @-> int @-> int @-> ptr mjrContext @-> returning void)


  (** Free resources in custom OpenGL context, set to default. *)
  let mjr_freeContext = foreign "mjr_freeContext" (ptr mjrContext @-> returning void)

  (** Upload texture to GPU, overwriting previous upload if any. *)
  let mjr_uploadTexture =
    foreign "mjr_uploadTexture" (ptr mjModel @-> ptr mjrContext @-> int @-> returning void)


  (** Upload mesh to GPU, overwriting previous upload if any. *)
  let mjr_uploadMesh =
    foreign "mjr_uploadMesh" (ptr mjModel @-> ptr mjrContext @-> int @-> returning void)


  (** Upload height field to GPU, overwriting previous upload if any. *)
  let mjr_uploadHField =
    foreign "mjr_uploadHField" (ptr mjModel @-> ptr mjrContext @-> int @-> returning void)


  (** Make con->currentBuffer current again. *)
  let mjr_restoreBuffer = foreign "mjr_restoreBuffer" (ptr mjrContext @-> returning void)

  (** Set OpenGL framebuffer for rendering: mjFB_WINDOW or mjFB_OFFSCREEN.
 If only one buffer is available, set that buffer and ignore framebuffer argument. *)
  let mjr_setBuffer = foreign "mjr_setBuffer" (int @-> ptr mjrContext @-> returning void)

  (** Read pixels from current OpenGL framebuffer to client buffer.
 Viewport is in OpenGL framebuffer; client buffer starts at (0,0). *)
  let mjr_readPixels =
    foreign
      "mjr_readPixels"
      (ptr uchar @-> ptr float @-> mjrRect @-> ptr mjrContext @-> returning void)


  (** Draw pixels from client buffer to current OpenGL framebuffer.
 Viewport is in OpenGL framebuffer; client buffer starts at (0,0). *)
  let mjr_drawPixels =
    foreign
      "mjr_drawPixels"
      (ptr uchar @-> ptr float @-> mjrRect @-> ptr mjrContext @-> returning void)


  (** Blit from src viewpoint in current framebuffer to dst viewport in other framebuffer.
 If src, dst have different size and flg_depth==0, color is interpolated with GL_LINEAR. *)
  let mjr_blitBuffer =
    foreign
      "mjr_blitBuffer"
      (mjrRect @-> mjrRect @-> int @-> int @-> ptr mjrContext @-> returning void)


  (** Set Aux buffer for custom OpenGL rendering (call restoreBuffer when done). *)
  let mjr_setAux = foreign "mjr_setAux" (int @-> ptr mjrContext @-> returning void)

  (** Blit from Aux buffer to con->currentBuffer. *)
  let mjr_blitAux =
    foreign
      "mjr_blitAux"
      (int @-> mjrRect @-> int @-> int @-> ptr mjrContext @-> returning void)


  (** Draw text at (x,y) in relative coordinates; font is mjtFont. *)
  let mjr_text =
    foreign
      "mjr_text"
      (int
      @-> string
      @-> ptr mjrContext
      @-> float
      @-> float
      @-> float
      @-> float
      @-> float
      @-> returning void)


  (** Draw text overlay; font is mjtFont; gridpos is mjtGridPos. *)
  let mjr_overlay =
    foreign
      "mjr_overlay"
      (int @-> int @-> mjrRect @-> string @-> string @-> ptr mjrContext @-> returning void)


  (** Get maximum viewport for active buffer. *)
  let mjr_maxViewport = foreign "mjr_maxViewport" (ptr mjrContext @-> returning mjrRect)

  (** Draw rectangle. *)
  let mjr_rectangle =
    foreign
      "mjr_rectangle"
      (mjrRect @-> float @-> float @-> float @-> float @-> returning void)


  (** Draw rectangle with centered text. *)
  let mjr_label =
    foreign
      "mjr_label"
      (mjrRect
      @-> int
      @-> string
      @-> float
      @-> float
      @-> float
      @-> float
      @-> float
      @-> float
      @-> float
      @-> ptr mjrContext
      @-> returning void)


  (** Draw 2D figure. *)
  let mjr_figure =
    foreign "mjr_figure" (mjrRect @-> ptr mjvFigure @-> ptr mjrContext @-> returning void)


  (** Render 3D scene. *)
  let mjr_render =
    foreign "mjr_render" (mjrRect @-> ptr mjvScene @-> ptr mjrContext @-> returning void)


  (** Call glFinish. *)
  let mjr_finish = foreign "mjr_finish" (void @-> returning void)

  (** Call glGetError and return result. *)
  let mjr_getError = foreign "mjr_getError" (void @-> returning int)

  (** Find first rectangle containing mouse, -1: not found. *)
  let mjr_findRect =
    foreign "mjr_findRect" (int @-> int @-> int @-> ptr mjrRect @-> returning int)


  (** ---------------------- UI framework --------------------------------------------------- *)

  (** ---------------------- Error and memory ----------------------------------------------- *)

  (** Main error function; does not return to caller. *)
  let mju_error = foreign "mju_error" (string @-> returning void)

  (** Error function with int argument; msg is a printf format string. *)
  let mju_error_i = foreign "mju_error_i" (string @-> int @-> returning void)

  (** Error function with string argument. *)
  let mju_error_s = foreign "mju_error_s" (string @-> string @-> returning void)

  (** Main warning function; returns to caller. *)
  let mju_warning = foreign "mju_warning" (string @-> returning void)

  (** Warning function with int argument. *)
  let mju_warning_i = foreign "mju_warning_i" (string @-> int @-> returning void)

  (** Warning function with string argument. *)
  let mju_warning_s = foreign "mju_warning_s" (string @-> string @-> returning void)

  (** Clear user error and memory handlers. *)
  let mju_clearHandlers = foreign "mju_clearHandlers" (void @-> returning void)

  (** Allocate memory; byte-align on 8; pad size to multiple of 8. *)
  let mju_malloc = foreign "mju_malloc" (size_t @-> returning (ptr void))

  (** Free memory, using free() by default. *)
  let mju_free = foreign "mju_free" (ptr void @-> returning void)

  (** High-level warning function: count warnings in mjData, print only the first. *)
  let mj_warning = foreign "mj_warning" (ptr mjData @-> int @-> int @-> returning void)

  (** Write [datetime, type: message] to MUJOCO_LOG.TXT. *)
  let mju_writeLog = foreign "mju_writeLog" (string @-> string @-> returning void)

  (** ---------------------- Standard math -------------------------------------------------- *)

  (** ------------------------------ Vector math -------------------------------------------- *)

  (** Set res = 0. *)
  let mju_zero3 = foreign "mju_zero3" (ptr mjtNum @-> returning void)

  (** Set res = vec. *)
  let mju_copy3 = foreign "mju_copy3" (ptr mjtNum @-> ptr mjtNum @-> returning void)

  (** Set res = vec*scl. *)
  let mju_scl3 =
    foreign "mju_scl3" (ptr mjtNum @-> ptr mjtNum @-> mjtNum @-> returning void)


  (** Set res = vec1 + vec2. *)
  let mju_add3 =
    foreign "mju_add3" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  (** Set res = vec1 - vec2. *)
  let mju_sub3 =
    foreign "mju_sub3" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  (** Set res = res + vec. *)
  let mju_addTo3 = foreign "mju_addTo3" (ptr mjtNum @-> ptr mjtNum @-> returning void)

  (** Set res = res - vec. *)
  let mju_subFrom3 = foreign "mju_subFrom3" (ptr mjtNum @-> ptr mjtNum @-> returning void)

  (** Set res = res + vec*scl. *)
  let mju_addToScl3 =
    foreign "mju_addToScl3" (ptr mjtNum @-> ptr mjtNum @-> mjtNum @-> returning void)


  (** Set res = vec1 + vec2*scl. *)
  let mju_addScl3 =
    foreign
      "mju_addScl3"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> mjtNum @-> returning void)


  (** Normalize vector, return length before normalization. *)
  let mju_normalize3 = foreign "mju_normalize3" (ptr mjtNum @-> returning mjtNum)

  (** Return vector length (without normalizing the vector). *)
  let mju_norm3 = foreign "mju_norm3" (ptr mjtNum @-> returning mjtNum)

  (** Return dot-product of vec1 and vec2. *)
  let mju_dot3 = foreign "mju_dot3" (ptr mjtNum @-> ptr mjtNum @-> returning mjtNum)

  (** Return Cartesian distance between 3D vectors pos1 and pos2. *)
  let mju_dist3 = foreign "mju_dist3" (ptr mjtNum @-> ptr mjtNum @-> returning mjtNum)

  (** Multiply vector by 3D rotation matrix: res = mat * vec. *)
  let mju_rotVecMat =
    foreign "mju_rotVecMat" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  (** Multiply vector by transposed 3D rotation matrix: res = mat' * vec. *)
  let mju_rotVecMatT =
    foreign "mju_rotVecMatT" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  (** Compute cross-product: res = cross(a, b). *)
  let mju_cross =
    foreign "mju_cross" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  (** Set res = 0. *)
  let mju_zero4 = foreign "mju_zero4" (ptr mjtNum @-> returning void)

  (** Set res = (1,0,0,0). *)
  let mju_unit4 = foreign "mju_unit4" (ptr mjtNum @-> returning void)

  (** Set res = vec. *)
  let mju_copy4 = foreign "mju_copy4" (ptr mjtNum @-> ptr mjtNum @-> returning void)

  (** Normalize vector, return length before normalization. *)
  let mju_normalize4 = foreign "mju_normalize4" (ptr mjtNum @-> returning mjtNum)

  (** Set res = 0. *)
  let mju_zero = foreign "mju_zero" (ptr mjtNum @-> int @-> returning void)

  (** Set res = vec. *)
  let mju_copy = foreign "mju_copy" (ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)

  (** Return sum(vec). *)
  let mju_sum = foreign "mju_sum" (ptr mjtNum @-> int @-> returning mjtNum)

  (** Return L1 norm: sum(abs(vec)). *)
  let mju_L1 = foreign "mju_L1" (ptr mjtNum @-> int @-> returning mjtNum)

  (** Set res = vec*scl. *)
  let mju_scl =
    foreign "mju_scl" (ptr mjtNum @-> ptr mjtNum @-> mjtNum @-> int @-> returning void)


  (** Set res = vec1 + vec2. *)
  let mju_add =
    foreign "mju_add" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  (** Set res = vec1 - vec2. *)
  let mju_sub =
    foreign "mju_sub" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  (** Set res = res + vec. *)
  let mju_addTo =
    foreign "mju_addTo" (ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  (** Set res = res - vec. *)
  let mju_subFrom =
    foreign "mju_subFrom" (ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  (** Set res = res + vec*scl. *)
  let mju_addToScl =
    foreign
      "mju_addToScl"
      (ptr mjtNum @-> ptr mjtNum @-> mjtNum @-> int @-> returning void)


  (** Set res = vec1 + vec2*scl. *)
  let mju_addScl =
    foreign
      "mju_addScl"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> mjtNum @-> int @-> returning void)


  (** Normalize vector, return length before normalization. *)
  let mju_normalize = foreign "mju_normalize" (ptr mjtNum @-> int @-> returning mjtNum)

  (** Return vector length (without normalizing vector). *)
  let mju_norm = foreign "mju_norm" (ptr mjtNum @-> int @-> returning mjtNum)

  (** Return dot-product of vec1 and vec2. *)
  let mju_dot = foreign "mju_dot" (ptr mjtNum @-> ptr mjtNum @-> int @-> returning mjtNum)

  (** Multiply matrix and vector: res = mat * vec. *)
  let mju_mulMatVec =
    foreign
      "mju_mulMatVec"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> int @-> returning void)


  (** Multiply transposed matrix and vector: res = mat' * vec. *)
  let mju_mulMatTVec =
    foreign
      "mju_mulMatTVec"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> int @-> returning void)


  (** Transpose matrix: res = mat'. *)
  let mju_transpose =
    foreign "mju_transpose" (ptr mjtNum @-> ptr mjtNum @-> int @-> int @-> returning void)


  (** Multiply matrices: res = mat1 * mat2. *)
  let mju_mulMatMat =
    foreign
      "mju_mulMatMat"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> int @-> int @-> returning void)


  (** Multiply matrices, second argument transposed: res = mat1 * mat2'. *)
  let mju_mulMatMatT =
    foreign
      "mju_mulMatMatT"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> int @-> int @-> returning void)


  (** Multiply matrices, first argument transposed: res = mat1' * mat2. *)
  let mju_mulMatTMat =
    foreign
      "mju_mulMatTMat"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> int @-> int @-> returning void)


  (** Set res = mat' * diag * mat if diag is not NULL, and res = mat' * mat otherwise. *)
  let mju_sqrMatTD =
    foreign
      "mju_sqrMatTD"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> int @-> returning void)


  (** Coordinate transform of 6D motion or force vector in rotation:translation format.
 rotnew2old is 3-by-3, NULL means no rotation; flg_force specifies force or motion type. *)
  let mju_transformSpatial =
    foreign
      "mju_transformSpatial"
      (ptr mjtNum
      @-> ptr mjtNum
      @-> int
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> returning void)


  (** ---------------------- Quaternions ---------------------------------------------------- *)

  (** Rotate vector by quaternion. *)
  let mju_rotVecQuat =
    foreign "mju_rotVecQuat" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  (** Conjugate quaternion, corresponding to opposite rotation. *)
  let mju_negQuat = foreign "mju_negQuat" (ptr mjtNum @-> ptr mjtNum @-> returning void)

  (** Muiltiply quaternions. *)
  let mju_mulQuat =
    foreign "mju_mulQuat" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  (** Muiltiply quaternion and axis. *)
  let mju_mulQuatAxis =
    foreign "mju_mulQuatAxis" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  (** Convert axisAngle to quaternion. *)
  let mju_axisAngle2Quat =
    foreign "mju_axisAngle2Quat" (ptr mjtNum @-> ptr mjtNum @-> mjtNum @-> returning void)


  (** Convert quaternion (corresponding to orientation difference) to 3D velocity. *)
  let mju_quat2Vel =
    foreign "mju_quat2Vel" (ptr mjtNum @-> ptr mjtNum @-> mjtNum @-> returning void)


  (** Subtract quaternions, express as 3D velocity: qb*quat(res) = qa. *)
  let mju_subQuat =
    foreign "mju_subQuat" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  (** Convert quaternion to 3D rotation matrix. *)
  let mju_quat2Mat = foreign "mju_quat2Mat" (ptr mjtNum @-> ptr mjtNum @-> returning void)

  (** Convert 3D rotation matrix to quaterion. *)
  let mju_mat2Quat = foreign "mju_mat2Quat" (ptr mjtNum @-> ptr mjtNum @-> returning void)

  (** Compute time-derivative of quaternion, given 3D rotational velocity. *)
  let mju_derivQuat =
    foreign "mju_derivQuat" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  (** Integrate quaterion given 3D angular velocity. *)
  let mju_quatIntegrate =
    foreign "mju_quatIntegrate" (ptr mjtNum @-> ptr mjtNum @-> mjtNum @-> returning void)


  (** Construct quaternion performing rotation from z-axis to given vector. *)
  let mju_quatZ2Vec =
    foreign "mju_quatZ2Vec" (ptr mjtNum @-> ptr mjtNum @-> returning void)


  (** ---------------------- Poses ---------------------------------------------------------- *)

  (** Multiply two poses. *)
  let mju_mulPose =
    foreign
      "mju_mulPose"
      (ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> returning void)


  (** Conjugate pose, corresponding to the opposite spatial transformation. *)
  let mju_negPose =
    foreign
      "mju_negPose"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  (** Transform vector by pose. *)
  let mju_trnVecPose =
    foreign
      "mju_trnVecPose"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  (** ---------------------- Decompositions -------------------------------------------------- *)

  (** Cholesky decomposition: mat = L*L'; return rank. *)
  let mju_cholFactor =
    foreign "mju_cholFactor" (ptr mjtNum @-> int @-> mjtNum @-> returning int)


  (** Solve mat * res = vec, where mat is Cholesky-factorized *)
  let mju_cholSolve =
    foreign
      "mju_cholSolve"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  (** Cholesky rank-one update: L*L' +/- x*x'; return rank. *)
  let mju_cholUpdate =
    foreign "mju_cholUpdate" (ptr mjtNum @-> ptr mjtNum @-> int @-> int @-> returning int)


  (** Eigenvalue decomposition of symmetric 3x3 matrix. *)
  let mju_eig3 =
    foreign
      "mju_eig3"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning int)


  (** ---------------------- Miscellaneous -------------------------------------------------- *)

  (** Muscle active force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax). *)
  let mju_muscleGain =
    foreign
      "mju_muscleGain"
      (mjtNum @-> mjtNum @-> ptr mjtNum @-> mjtNum @-> ptr mjtNum @-> returning mjtNum)


  (** Muscle passive force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax). *)
  let mju_muscleBias =
    foreign
      "mju_muscleBias"
      (mjtNum @-> ptr mjtNum @-> mjtNum @-> ptr mjtNum @-> returning mjtNum)


  (** Muscle activation dynamics, prm = (tau_act, tau_deact). *)
  let mju_muscleDynamics =
    foreign "mju_muscleDynamics" (mjtNum @-> mjtNum @-> ptr mjtNum @-> returning mjtNum)


  (** Convert contact force to pyramid representation. *)
  let mju_encodePyramid =
    foreign
      "mju_encodePyramid"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  (** Convert pyramid representation to contact force. *)
  let mju_decodePyramid =
    foreign
      "mju_decodePyramid"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  (** Integrate spring-damper analytically, return pos(dt). *)
  let mju_springDamper =
    foreign
      "mju_springDamper"
      (mjtNum @-> mjtNum @-> mjtNum @-> mjtNum @-> mjtNum @-> returning mjtNum)


  (** Return min(a,b) with single evaluation of a and b. *)
  let mju_min = foreign "mju_min" (mjtNum @-> mjtNum @-> returning mjtNum)

  (** Return max(a,b) with single evaluation of a and b. *)
  let mju_max = foreign "mju_max" (mjtNum @-> mjtNum @-> returning mjtNum)

  (** Return sign of x: +1, -1 or 0. *)
  let mju_sign = foreign "mju_sign" (mjtNum @-> returning mjtNum)

  (** Round x to nearest integer. *)
  let mju_round = foreign "mju_round" (mjtNum @-> returning int)

  (** Convert type name to type id (mjtObj). *)
  let mju_str2Type = foreign "mju_str2Type" (string @-> returning int)

  (** Return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise. Used by check functions. *)
  let mju_isBad = foreign "mju_isBad" (mjtNum @-> returning int)

  (** Return 1 if all elements are 0. *)
  let mju_isZero = foreign "mju_isZero" (ptr mjtNum @-> int @-> returning int)

  (** Standard normal random number generator (optional second number). *)
  let mju_standardNormal = foreign "mju_standardNormal" (ptr mjtNum @-> returning mjtNum)

  (** Convert from float to mjtNum. *)
  let mju_f2n = foreign "mju_f2n" (ptr mjtNum @-> ptr float @-> int @-> returning void)

  (** Convert from mjtNum to float. *)
  let mju_n2f = foreign "mju_n2f" (ptr float @-> ptr mjtNum @-> int @-> returning void)

  (** Convert from double to mjtNum. *)
  let mju_d2n = foreign "mju_d2n" (ptr mjtNum @-> ptr double @-> int @-> returning void)

  (** Convert from mjtNum to double. *)
  let mju_n2d = foreign "mju_n2d" (ptr double @-> ptr mjtNum @-> int @-> returning void)

  (** Insertion sort, resulting list is in increasing order. *)
  let mju_insertionSort =
    foreign "mju_insertionSort" (ptr mjtNum @-> int @-> returning void)


  (** Integer insertion sort, resulting list is in increasing order. *)
  let mju_insertionSortInt =
    foreign "mju_insertionSortInt" (ptr int @-> int @-> returning void)


  (** Generate Halton sequence. *)
  let mju_Halton = foreign "mju_Halton" (int @-> int @-> returning mjtNum)

  (** Call strncpy, then set dst[n-1] = 0. *)
  let mju_strncpy = foreign "mju_strncpy" (char @-> char @-> int @-> returning string)

  (** Sigmoid function over 0<=x<=1 constructed from half-quadratics. *)
  let mju_sigmoid = foreign "mju_sigmoid" (mjtNum @-> returning mjtNum)
end
