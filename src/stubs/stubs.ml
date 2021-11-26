open Ctypes
module Typs = Typs
open Typs

module Bindings (F : FOREIGN) = struct
  open F

  let mj_loadXML =
    foreign
      "mj_loadXML"
      (string @-> ptr mjVFS @-> string @-> int @-> returning (ptr mjModel))


  let mj_deleteModel = foreign "mj_deleteModel" (ptr mjModel @-> returning void)
  let mj_makeData = foreign "mj_makeData" (ptr mjModel @-> returning (ptr mjData))
  let mj_deleteData = foreign "mj_deleteData" (ptr mjData @-> returning void)
  let mj_resetData = foreign "mj_resetData" (ptr mjModel @-> ptr mjData @-> returning void)
  let mj_forward = foreign "mj_forward" (ptr mjModel @-> ptr mjData @-> returning void)
  let mj_step = foreign "mj_step" (ptr mjModel @-> ptr mjData @-> returning void)

  let mjv_makeScene =
    foreign "mjv_makeScene" (ptr mjModel @-> ptr mjvScene @-> int @-> returning void)


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


  let mjv_defaultCamera = foreign "mjv_defaultCamera" (ptr mjvCamera @-> returning void)
  let mjv_defaultOption = foreign "mjv_defaultOption" (ptr mjvOption @-> returning void)
  let mjv_defaultScene = foreign "mjv_defaultScene" (ptr mjvScene @-> returning void)

  let mjv_moveCamera =
    foreign
      "mjv_moveCamera"
      (ptr mjModel
      @-> mjtMouse
      @-> double
      @-> double
      @-> ptr mjvScene
      @-> ptr mjvCamera
      @-> returning void)


  let mjr_defaultContext = foreign "mjr_defaultContext" (ptr mjrContext @-> returning void)

  let mjr_makeContext =
    foreign "mjr_makeContext" (ptr mjModel @-> ptr mjrContext @-> int @-> returning void)


  let mjr_render =
    foreign "mjr_render" (mjrRect @-> ptr mjvScene @-> ptr mjrContext @-> returning void)
end
