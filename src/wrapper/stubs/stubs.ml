(* THIS FILE IS GENERATED AUTOMATICALLY, DO NOT EDIT BY HAND *)

open Ctypes
module Typs = Typs
open Typs

module Bindings (F : FOREIGN) = struct
  open F

  let mj_activate = foreign "mj_activate" (string @-> returning int)
  let mj_deactivate = foreign "mj_deactivate" (void @-> returning void)
  let mj_defaultVFS = foreign "mj_defaultVFS" (ptr mjVFS @-> returning void)

  let mj_addFileVFS =
    foreign "mj_addFileVFS" (ptr mjVFS @-> string @-> string @-> returning int)


  let mj_makeEmptyFileVFS =
    foreign "mj_makeEmptyFileVFS" (ptr mjVFS @-> string @-> int @-> returning int)


  let mj_findFileVFS = foreign "mj_findFileVFS" (ptr mjVFS @-> string @-> returning int)

  let mj_deleteFileVFS =
    foreign "mj_deleteFileVFS" (ptr mjVFS @-> string @-> returning int)


  let mj_deleteVFS = foreign "mj_deleteVFS" (ptr mjVFS @-> returning void)

  let mj_loadXML =
    foreign
      "mj_loadXML"
      (string @-> ptr mjVFS @-> string @-> int @-> returning (ptr mjModel))


  let mj_saveLastXML =
    foreign "mj_saveLastXML" (string @-> ptr mjModel @-> string @-> int @-> returning int)


  let mj_freeLastXML = foreign "mj_freeLastXML" (void @-> returning void)

  let mj_printSchema =
    foreign "mj_printSchema" (string @-> string @-> int @-> int @-> int @-> returning int)


  let mj_step = foreign "mj_step" (ptr mjModel @-> ptr mjData @-> returning void)
  let mj_step1 = foreign "mj_step1" (ptr mjModel @-> ptr mjData @-> returning void)
  let mj_step2 = foreign "mj_step2" (ptr mjModel @-> ptr mjData @-> returning void)
  let mj_forward = foreign "mj_forward" (ptr mjModel @-> ptr mjData @-> returning void)
  let mj_inverse = foreign "mj_inverse" (ptr mjModel @-> ptr mjData @-> returning void)

  let mj_forwardSkip =
    foreign
      "mj_forwardSkip"
      (ptr mjModel @-> ptr mjData @-> int @-> int @-> returning void)


  let mj_inverseSkip =
    foreign
      "mj_inverseSkip"
      (ptr mjModel @-> ptr mjData @-> int @-> int @-> returning void)


  let mj_defaultLROpt = foreign "mj_defaultLROpt" (ptr mjLROpt @-> returning void)

  let mj_defaultSolRefImp =
    foreign "mj_defaultSolRefImp" (ptr mjtNum @-> ptr mjtNum @-> returning void)


  let mj_defaultOption = foreign "mj_defaultOption" (ptr mjOption @-> returning void)
  let mj_defaultVisual = foreign "mj_defaultVisual" (ptr mjVisual @-> returning void)

  let mj_copyModel =
    foreign "mj_copyModel" (ptr mjModel @-> ptr mjModel @-> returning (ptr mjModel))


  let mj_saveModel =
    foreign "mj_saveModel" (ptr mjModel @-> string @-> ptr void @-> int @-> returning void)


  let mj_loadModel =
    foreign "mj_loadModel" (string @-> ptr mjVFS @-> returning (ptr mjModel))


  let mj_deleteModel = foreign "mj_deleteModel" (ptr mjModel @-> returning void)
  let mj_sizeModel = foreign "mj_sizeModel" (ptr mjModel @-> returning int)
  let mj_makeData = foreign "mj_makeData" (ptr mjModel @-> returning (ptr mjData))

  let mj_copyData =
    foreign
      "mj_copyData"
      (ptr mjData @-> ptr mjModel @-> ptr mjData @-> returning (ptr mjData))


  let mj_resetData = foreign "mj_resetData" (ptr mjModel @-> ptr mjData @-> returning void)

  let mj_resetDataDebug =
    foreign "mj_resetDataDebug" (ptr mjModel @-> ptr mjData @-> uchar @-> returning void)


  let mj_resetDataKeyframe =
    foreign "mj_resetDataKeyframe" (ptr mjModel @-> ptr mjData @-> int @-> returning void)


  let mj_stackAlloc =
    foreign "mj_stackAlloc" (ptr mjData @-> int @-> returning (ptr mjtNum))


  let mj_deleteData = foreign "mj_deleteData" (ptr mjData @-> returning void)
  let mj_resetCallbacks = foreign "mj_resetCallbacks" (void @-> returning void)
  let mj_setConst = foreign "mj_setConst" (ptr mjModel @-> ptr mjData @-> returning void)

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


  let mj_printModel = foreign "mj_printModel" (ptr mjModel @-> string @-> returning void)

  let mj_printData =
    foreign "mj_printData" (ptr mjModel @-> ptr mjData @-> string @-> returning void)


  let mju_printMat = foreign "mju_printMat" (ptr mjtNum @-> int @-> int @-> returning void)

  let mju_printMatSparse =
    foreign
      "mju_printMatSparse"
      (ptr mjtNum @-> int @-> ptr int @-> ptr int @-> ptr int @-> returning void)


  let mj_fwdPosition =
    foreign "mj_fwdPosition" (ptr mjModel @-> ptr mjData @-> returning void)


  let mj_fwdVelocity =
    foreign "mj_fwdVelocity" (ptr mjModel @-> ptr mjData @-> returning void)


  let mj_fwdActuation =
    foreign "mj_fwdActuation" (ptr mjModel @-> ptr mjData @-> returning void)


  let mj_fwdAcceleration =
    foreign "mj_fwdAcceleration" (ptr mjModel @-> ptr mjData @-> returning void)


  let mj_fwdConstraint =
    foreign "mj_fwdConstraint" (ptr mjModel @-> ptr mjData @-> returning void)


  let mj_Euler = foreign "mj_Euler" (ptr mjModel @-> ptr mjData @-> returning void)

  let mj_RungeKutta =
    foreign "mj_RungeKutta" (ptr mjModel @-> ptr mjData @-> int @-> returning void)


  let mj_invPosition =
    foreign "mj_invPosition" (ptr mjModel @-> ptr mjData @-> returning void)


  let mj_invVelocity =
    foreign "mj_invVelocity" (ptr mjModel @-> ptr mjData @-> returning void)


  let mj_invConstraint =
    foreign "mj_invConstraint" (ptr mjModel @-> ptr mjData @-> returning void)


  let mj_compareFwdInv =
    foreign "mj_compareFwdInv" (ptr mjModel @-> ptr mjData @-> returning void)


  let mj_sensorPos = foreign "mj_sensorPos" (ptr mjModel @-> ptr mjData @-> returning void)
  let mj_sensorVel = foreign "mj_sensorVel" (ptr mjModel @-> ptr mjData @-> returning void)
  let mj_sensorAcc = foreign "mj_sensorAcc" (ptr mjModel @-> ptr mjData @-> returning void)
  let mj_energyPos = foreign "mj_energyPos" (ptr mjModel @-> ptr mjData @-> returning void)
  let mj_energyVel = foreign "mj_energyVel" (ptr mjModel @-> ptr mjData @-> returning void)
  let mj_checkPos = foreign "mj_checkPos" (ptr mjModel @-> ptr mjData @-> returning void)
  let mj_checkVel = foreign "mj_checkVel" (ptr mjModel @-> ptr mjData @-> returning void)
  let mj_checkAcc = foreign "mj_checkAcc" (ptr mjModel @-> ptr mjData @-> returning void)

  let mj_kinematics =
    foreign "mj_kinematics" (ptr mjModel @-> ptr mjData @-> returning void)


  let mj_comPos = foreign "mj_comPos" (ptr mjModel @-> ptr mjData @-> returning void)
  let mj_camlight = foreign "mj_camlight" (ptr mjModel @-> ptr mjData @-> returning void)
  let mj_tendon = foreign "mj_tendon" (ptr mjModel @-> ptr mjData @-> returning void)

  let mj_transmission =
    foreign "mj_transmission" (ptr mjModel @-> ptr mjData @-> returning void)


  let mj_crb = foreign "mj_crb" (ptr mjModel @-> ptr mjData @-> returning void)
  let mj_factorM = foreign "mj_factorM" (ptr mjModel @-> ptr mjData @-> returning void)

  let mj_solveM =
    foreign
      "mj_solveM"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  let mj_solveM2 =
    foreign
      "mj_solveM2"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  let mj_comVel = foreign "mj_comVel" (ptr mjModel @-> ptr mjData @-> returning void)
  let mj_passive = foreign "mj_passive" (ptr mjModel @-> ptr mjData @-> returning void)

  let mj_subtreeVel =
    foreign "mj_subtreeVel" (ptr mjModel @-> ptr mjData @-> returning void)


  let mj_rne =
    foreign "mj_rne" (ptr mjModel @-> ptr mjData @-> int @-> ptr mjtNum @-> returning void)


  let mj_rnePostConstraint =
    foreign "mj_rnePostConstraint" (ptr mjModel @-> ptr mjData @-> returning void)


  let mj_collision = foreign "mj_collision" (ptr mjModel @-> ptr mjData @-> returning void)

  let mj_makeConstraint =
    foreign "mj_makeConstraint" (ptr mjModel @-> ptr mjData @-> returning void)


  let mj_projectConstraint =
    foreign "mj_projectConstraint" (ptr mjModel @-> ptr mjData @-> returning void)


  let mj_referenceConstraint =
    foreign "mj_referenceConstraint" (ptr mjModel @-> ptr mjData @-> returning void)


  let mj_constraintUpdate =
    foreign
      "mj_constraintUpdate"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  let mj_addContact =
    foreign
      "mj_addContact"
      (ptr mjModel @-> ptr mjData @-> ptr mjContact @-> returning int)


  let mj_isPyramidal = foreign "mj_isPyramidal" (ptr mjModel @-> returning int)
  let mj_isSparse = foreign "mj_isSparse" (ptr mjModel @-> returning int)
  let mj_isDual = foreign "mj_isDual" (ptr mjModel @-> returning int)

  let mj_mulJacVec =
    foreign
      "mj_mulJacVec"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  let mj_mulJacTVec =
    foreign
      "mj_mulJacTVec"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


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


  let mj_jacBody =
    foreign
      "mj_jacBody"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  let mj_jacBodyCom =
    foreign
      "mj_jacBodyCom"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  let mj_jacGeom =
    foreign
      "mj_jacGeom"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  let mj_jacSite =
    foreign
      "mj_jacSite"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


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


  let mj_name2id =
    foreign "mj_name2id" (ptr mjModel @-> mjtObj @-> string @-> returning int)


  let mj_fullM =
    foreign "mj_fullM" (ptr mjModel @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  let mj_mulM =
    foreign
      "mj_mulM"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  let mj_mulM2 =
    foreign
      "mj_mulM2"
      (ptr mjModel @-> ptr mjData @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


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


  let mj_contactForce =
    foreign
      "mj_contactForce"
      (ptr mjModel @-> ptr mjData @-> int @-> ptr mjtNum @-> returning void)


  let mj_differentiatePos =
    foreign
      "mj_differentiatePos"
      (ptr mjModel
      @-> ptr mjtNum
      @-> mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> returning void)


  let mj_integratePos =
    foreign
      "mj_integratePos"
      (ptr mjModel @-> ptr mjtNum @-> ptr mjtNum @-> mjtNum @-> returning void)


  let mj_normalizeQuat =
    foreign "mj_normalizeQuat" (ptr mjModel @-> ptr mjtNum @-> returning void)


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


  let mj_getTotalmass = foreign "mj_getTotalmass" (ptr mjModel @-> returning mjtNum)

  let mj_setTotalmass =
    foreign "mj_setTotalmass" (ptr mjModel @-> mjtNum @-> returning void)


  let mj_version = foreign "mj_version" (void @-> returning int)

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


  let mj_rayHfield =
    foreign
      "mj_rayHfield"
      (ptr mjModel
      @-> ptr mjData
      @-> int
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> returning mjtNum)


  let mj_rayMesh =
    foreign
      "mj_rayMesh"
      (ptr mjModel
      @-> ptr mjData
      @-> int
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> returning mjtNum)


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


  let mjv_defaultCamera = foreign "mjv_defaultCamera" (ptr mjvCamera @-> returning void)
  let mjv_defaultPerturb = foreign "mjv_defaultPerturb" (ptr mjvPerturb @-> returning void)

  let mjv_room2model =
    foreign
      "mjv_room2model"
      (ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjvScene
      @-> returning void)


  let mjv_model2room =
    foreign
      "mjv_model2room"
      (ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjvScene
      @-> returning void)


  let mjv_cameraInModel =
    foreign
      "mjv_cameraInModel"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> ptr mjvScene @-> returning void)


  let mjv_cameraInRoom =
    foreign
      "mjv_cameraInRoom"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> ptr mjvScene @-> returning void)


  let mjv_frustumHeight = foreign "mjv_frustumHeight" (ptr mjvScene @-> returning mjtNum)

  let mjv_alignToCamera =
    foreign
      "mjv_alignToCamera"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  let mjv_moveCamera =
    foreign
      "mjv_moveCamera"
      (ptr mjModel
      @-> mjtMouse
      @-> mjtNum
      @-> mjtNum
      @-> ptr mjvScene
      @-> ptr mjvCamera
      @-> returning void)


  let mjv_movePerturb =
    foreign
      "mjv_movePerturb"
      (ptr mjModel
      @-> ptr mjData
      @-> mjtMouse
      @-> mjtNum
      @-> mjtNum
      @-> ptr mjvScene
      @-> ptr mjvPerturb
      @-> returning void)


  let mjv_moveModel =
    foreign
      "mjv_moveModel"
      (ptr mjModel
      @-> mjtMouse
      @-> mjtNum
      @-> mjtNum
      @-> ptr mjtNum
      @-> ptr mjvScene
      @-> returning void)


  let mjv_initPerturb =
    foreign
      "mjv_initPerturb"
      (ptr mjModel @-> ptr mjData @-> ptr mjvScene @-> ptr mjvPerturb @-> returning void)


  let mjv_applyPerturbPose =
    foreign
      "mjv_applyPerturbPose"
      (ptr mjModel @-> ptr mjData @-> ptr mjvPerturb @-> int @-> returning void)


  let mjv_applyPerturbForce =
    foreign
      "mjv_applyPerturbForce"
      (ptr mjModel @-> ptr mjData @-> ptr mjvPerturb @-> returning void)


  let mjv_averageCamera =
    foreign
      "mjv_averageCamera"
      (ptr mjvGLCamera @-> ptr mjvGLCamera @-> returning mjvGLCamera)


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


  let mjv_defaultOption = foreign "mjv_defaultOption" (ptr mjvOption @-> returning void)
  let mjv_defaultFigure = foreign "mjv_defaultFigure" (ptr mjvFigure @-> returning void)

  let mjv_initGeom =
    foreign
      "mjv_initGeom"
      (ptr mjvGeom
      @-> mjtObj
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr mjtNum
      @-> ptr float
      @-> returning void)


  let mjv_makeConnector =
    foreign
      "mjv_makeConnector"
      (ptr mjvGeom
      @-> mjtObj
      @-> mjtNum
      @-> mjtNum
      @-> mjtNum
      @-> mjtNum
      @-> mjtNum
      @-> mjtNum
      @-> mjtNum
      @-> returning void)


  let mjv_defaultScene = foreign "mjv_defaultScene" (ptr mjvScene @-> returning void)

  let mjv_makeScene =
    foreign "mjv_makeScene" (ptr mjModel @-> ptr mjvScene @-> int @-> returning void)


  let mjv_freeScene = foreign "mjv_freeScene" (ptr mjvScene @-> returning void)

  let mjv_updateScene =
    foreign
      "mjv_updateScene"
      (ptr mjModel
      @-> ptr mjData
      @-> ptr mjvOption
      @-> ptr mjvPerturb
      @-> ptr mjvCamera
      @-> mjtCatBit
      @-> ptr mjvScene
      @-> returning void)


  let mjv_addGeoms =
    foreign
      "mjv_addGeoms"
      (ptr mjModel
      @-> ptr mjData
      @-> ptr mjvOption
      @-> ptr mjvPerturb
      @-> mjtCatBit
      @-> ptr mjvScene
      @-> returning void)


  let mjv_makeLights =
    foreign
      "mjv_makeLights"
      (ptr mjModel @-> ptr mjData @-> ptr mjvScene @-> returning void)


  let mjv_updateCamera =
    foreign
      "mjv_updateCamera"
      (ptr mjModel @-> ptr mjData @-> ptr mjvCamera @-> ptr mjvScene @-> returning void)


  let mjv_updateSkin =
    foreign
      "mjv_updateSkin"
      (ptr mjModel @-> ptr mjData @-> ptr mjvScene @-> returning void)


  let mjr_defaultContext = foreign "mjr_defaultContext" (ptr mjrContext @-> returning void)

  let mjr_makeContext =
    foreign "mjr_makeContext" (ptr mjModel @-> ptr mjrContext @-> int @-> returning void)


  let mjr_changeFont = foreign "mjr_changeFont" (int @-> ptr mjrContext @-> returning void)

  let mjr_addAux =
    foreign
      "mjr_addAux"
      (int @-> int @-> int @-> int @-> ptr mjrContext @-> returning void)


  let mjr_freeContext = foreign "mjr_freeContext" (ptr mjrContext @-> returning void)

  let mjr_uploadTexture =
    foreign "mjr_uploadTexture" (ptr mjModel @-> ptr mjrContext @-> int @-> returning void)


  let mjr_uploadMesh =
    foreign "mjr_uploadMesh" (ptr mjModel @-> ptr mjrContext @-> int @-> returning void)


  let mjr_uploadHField =
    foreign "mjr_uploadHField" (ptr mjModel @-> ptr mjrContext @-> int @-> returning void)


  let mjr_restoreBuffer = foreign "mjr_restoreBuffer" (ptr mjrContext @-> returning void)
  let mjr_setBuffer = foreign "mjr_setBuffer" (int @-> ptr mjrContext @-> returning void)

  let mjr_readPixels =
    foreign
      "mjr_readPixels"
      (ptr uchar @-> ptr float @-> mjrRect @-> ptr mjrContext @-> returning void)


  let mjr_drawPixels =
    foreign
      "mjr_drawPixels"
      (ptr uchar @-> ptr float @-> mjrRect @-> ptr mjrContext @-> returning void)


  let mjr_blitBuffer =
    foreign
      "mjr_blitBuffer"
      (mjrRect @-> mjrRect @-> int @-> int @-> ptr mjrContext @-> returning void)


  let mjr_setAux = foreign "mjr_setAux" (int @-> ptr mjrContext @-> returning void)

  let mjr_blitAux =
    foreign
      "mjr_blitAux"
      (int @-> mjrRect @-> int @-> int @-> ptr mjrContext @-> returning void)


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


  let mjr_overlay =
    foreign
      "mjr_overlay"
      (int @-> int @-> mjrRect @-> string @-> string @-> ptr mjrContext @-> returning void)


  let mjr_maxViewport = foreign "mjr_maxViewport" (ptr mjrContext @-> returning mjrRect)

  let mjr_rectangle =
    foreign
      "mjr_rectangle"
      (mjrRect @-> float @-> float @-> float @-> float @-> returning void)


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


  let mjr_figure =
    foreign "mjr_figure" (mjrRect @-> ptr mjvFigure @-> ptr mjrContext @-> returning void)


  let mjr_render =
    foreign "mjr_render" (mjrRect @-> ptr mjvScene @-> ptr mjrContext @-> returning void)


  let mjr_finish = foreign "mjr_finish" (void @-> returning void)
  let mjr_getError = foreign "mjr_getError" (void @-> returning int)

  let mjr_findRect =
    foreign "mjr_findRect" (int @-> int @-> int @-> ptr mjrRect @-> returning int)


  let mju_error = foreign "mju_error" (string @-> returning void)
  let mju_error_i = foreign "mju_error_i" (string @-> int @-> returning void)
  let mju_error_s = foreign "mju_error_s" (string @-> string @-> returning void)
  let mju_warning = foreign "mju_warning" (string @-> returning void)
  let mju_warning_i = foreign "mju_warning_i" (string @-> int @-> returning void)
  let mju_warning_s = foreign "mju_warning_s" (string @-> string @-> returning void)
  let mju_clearHandlers = foreign "mju_clearHandlers" (void @-> returning void)
  let mju_malloc = foreign "mju_malloc" (size_t @-> returning (ptr void))
  let mju_free = foreign "mju_free" (ptr void @-> returning void)
  let mj_warning = foreign "mj_warning" (ptr mjData @-> int @-> int @-> returning void)
  let mju_writeLog = foreign "mju_writeLog" (string @-> string @-> returning void)
  let mju_zero3 = foreign "mju_zero3" (ptr mjtNum @-> returning void)
  let mju_copy3 = foreign "mju_copy3" (ptr mjtNum @-> ptr mjtNum @-> returning void)

  let mju_scl3 =
    foreign "mju_scl3" (ptr mjtNum @-> ptr mjtNum @-> mjtNum @-> returning void)


  let mju_add3 =
    foreign "mju_add3" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  let mju_sub3 =
    foreign "mju_sub3" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  let mju_addTo3 = foreign "mju_addTo3" (ptr mjtNum @-> ptr mjtNum @-> returning void)
  let mju_subFrom3 = foreign "mju_subFrom3" (ptr mjtNum @-> ptr mjtNum @-> returning void)

  let mju_addToScl3 =
    foreign "mju_addToScl3" (ptr mjtNum @-> ptr mjtNum @-> mjtNum @-> returning void)


  let mju_addScl3 =
    foreign
      "mju_addScl3"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> mjtNum @-> returning void)


  let mju_normalize3 = foreign "mju_normalize3" (ptr mjtNum @-> returning mjtNum)
  let mju_norm3 = foreign "mju_norm3" (ptr mjtNum @-> returning mjtNum)
  let mju_dot3 = foreign "mju_dot3" (ptr mjtNum @-> ptr mjtNum @-> returning mjtNum)
  let mju_dist3 = foreign "mju_dist3" (ptr mjtNum @-> ptr mjtNum @-> returning mjtNum)

  let mju_rotVecMat =
    foreign "mju_rotVecMat" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  let mju_rotVecMatT =
    foreign "mju_rotVecMatT" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  let mju_cross =
    foreign "mju_cross" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  let mju_zero4 = foreign "mju_zero4" (ptr mjtNum @-> returning void)
  let mju_unit4 = foreign "mju_unit4" (ptr mjtNum @-> returning void)
  let mju_copy4 = foreign "mju_copy4" (ptr mjtNum @-> ptr mjtNum @-> returning void)
  let mju_normalize4 = foreign "mju_normalize4" (ptr mjtNum @-> returning mjtNum)
  let mju_zero = foreign "mju_zero" (ptr mjtNum @-> int @-> returning void)
  let mju_copy = foreign "mju_copy" (ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)
  let mju_sum = foreign "mju_sum" (ptr mjtNum @-> int @-> returning mjtNum)
  let mju_L1 = foreign "mju_L1" (ptr mjtNum @-> int @-> returning mjtNum)

  let mju_scl =
    foreign "mju_scl" (ptr mjtNum @-> ptr mjtNum @-> mjtNum @-> int @-> returning void)


  let mju_add =
    foreign "mju_add" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  let mju_sub =
    foreign "mju_sub" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  let mju_addTo =
    foreign "mju_addTo" (ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  let mju_subFrom =
    foreign "mju_subFrom" (ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  let mju_addToScl =
    foreign
      "mju_addToScl"
      (ptr mjtNum @-> ptr mjtNum @-> mjtNum @-> int @-> returning void)


  let mju_addScl =
    foreign
      "mju_addScl"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> mjtNum @-> int @-> returning void)


  let mju_normalize = foreign "mju_normalize" (ptr mjtNum @-> int @-> returning mjtNum)
  let mju_norm = foreign "mju_norm" (ptr mjtNum @-> int @-> returning mjtNum)
  let mju_dot = foreign "mju_dot" (ptr mjtNum @-> ptr mjtNum @-> int @-> returning mjtNum)

  let mju_mulMatVec =
    foreign
      "mju_mulMatVec"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> int @-> returning void)


  let mju_mulMatTVec =
    foreign
      "mju_mulMatTVec"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> int @-> returning void)


  let mju_transpose =
    foreign "mju_transpose" (ptr mjtNum @-> ptr mjtNum @-> int @-> int @-> returning void)


  let mju_mulMatMat =
    foreign
      "mju_mulMatMat"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> int @-> int @-> returning void)


  let mju_mulMatMatT =
    foreign
      "mju_mulMatMatT"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> int @-> int @-> returning void)


  let mju_mulMatTMat =
    foreign
      "mju_mulMatTMat"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> int @-> int @-> returning void)


  let mju_sqrMatTD =
    foreign
      "mju_sqrMatTD"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> int @-> returning void)


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


  let mju_rotVecQuat =
    foreign "mju_rotVecQuat" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  let mju_negQuat = foreign "mju_negQuat" (ptr mjtNum @-> ptr mjtNum @-> returning void)

  let mju_mulQuat =
    foreign "mju_mulQuat" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  let mju_mulQuatAxis =
    foreign "mju_mulQuatAxis" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  let mju_axisAngle2Quat =
    foreign "mju_axisAngle2Quat" (ptr mjtNum @-> ptr mjtNum @-> mjtNum @-> returning void)


  let mju_quat2Vel =
    foreign "mju_quat2Vel" (ptr mjtNum @-> ptr mjtNum @-> mjtNum @-> returning void)


  let mju_subQuat =
    foreign "mju_subQuat" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  let mju_quat2Mat = foreign "mju_quat2Mat" (ptr mjtNum @-> ptr mjtNum @-> returning void)
  let mju_mat2Quat = foreign "mju_mat2Quat" (ptr mjtNum @-> ptr mjtNum @-> returning void)

  let mju_derivQuat =
    foreign "mju_derivQuat" (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  let mju_quatIntegrate =
    foreign "mju_quatIntegrate" (ptr mjtNum @-> ptr mjtNum @-> mjtNum @-> returning void)


  let mju_quatZ2Vec =
    foreign "mju_quatZ2Vec" (ptr mjtNum @-> ptr mjtNum @-> returning void)


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


  let mju_negPose =
    foreign
      "mju_negPose"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  let mju_trnVecPose =
    foreign
      "mju_trnVecPose"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning void)


  let mju_cholFactor =
    foreign "mju_cholFactor" (ptr mjtNum @-> int @-> mjtNum @-> returning int)


  let mju_cholSolve =
    foreign
      "mju_cholSolve"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  let mju_cholUpdate =
    foreign "mju_cholUpdate" (ptr mjtNum @-> ptr mjtNum @-> int @-> int @-> returning int)


  let mju_eig3 =
    foreign
      "mju_eig3"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> returning int)


  let mju_muscleGain =
    foreign
      "mju_muscleGain"
      (mjtNum @-> mjtNum @-> ptr mjtNum @-> mjtNum @-> ptr mjtNum @-> returning mjtNum)


  let mju_muscleBias =
    foreign
      "mju_muscleBias"
      (mjtNum @-> ptr mjtNum @-> mjtNum @-> ptr mjtNum @-> returning mjtNum)


  let mju_muscleDynamics =
    foreign "mju_muscleDynamics" (mjtNum @-> mjtNum @-> ptr mjtNum @-> returning mjtNum)


  let mju_encodePyramid =
    foreign
      "mju_encodePyramid"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  let mju_decodePyramid =
    foreign
      "mju_decodePyramid"
      (ptr mjtNum @-> ptr mjtNum @-> ptr mjtNum @-> int @-> returning void)


  let mju_springDamper =
    foreign
      "mju_springDamper"
      (mjtNum @-> mjtNum @-> mjtNum @-> mjtNum @-> mjtNum @-> returning mjtNum)


  let mju_min = foreign "mju_min" (mjtNum @-> mjtNum @-> returning mjtNum)
  let mju_max = foreign "mju_max" (mjtNum @-> mjtNum @-> returning mjtNum)
  let mju_sign = foreign "mju_sign" (mjtNum @-> returning mjtNum)
  let mju_round = foreign "mju_round" (mjtNum @-> returning int)
  let mju_str2Type = foreign "mju_str2Type" (string @-> returning int)
  let mju_isBad = foreign "mju_isBad" (mjtNum @-> returning int)
  let mju_isZero = foreign "mju_isZero" (ptr mjtNum @-> int @-> returning int)
  let mju_standardNormal = foreign "mju_standardNormal" (ptr mjtNum @-> returning mjtNum)
  let mju_f2n = foreign "mju_f2n" (ptr mjtNum @-> ptr float @-> int @-> returning void)
  let mju_n2f = foreign "mju_n2f" (ptr float @-> ptr mjtNum @-> int @-> returning void)
  let mju_d2n = foreign "mju_d2n" (ptr mjtNum @-> ptr double @-> int @-> returning void)
  let mju_n2d = foreign "mju_n2d" (ptr double @-> ptr mjtNum @-> int @-> returning void)

  let mju_insertionSort =
    foreign "mju_insertionSort" (ptr mjtNum @-> int @-> returning void)


  let mju_insertionSortInt =
    foreign "mju_insertionSortInt" (ptr int @-> int @-> returning void)


  let mju_Halton = foreign "mju_Halton" (int @-> int @-> returning mjtNum)
  let mju_strncpy = foreign "mju_strncpy" (char @-> char @-> int @-> returning string)
  let mju_sigmoid = foreign "mju_sigmoid" (mjtNum @-> returning mjtNum)
end
