open Wrapper

type 'a cptr = 'a Ctypes.structure Ctypes_static.ptr
type 'a cstruct = 'a Ctypes.structure
type model = Typs.mjModel cptr
type data = Typs.mjData cptr
type vcamera = Typs.mjvCamera cstruct
type vscene = Typs.mjvScene cstruct
type rrect = Typs.mjrRect cstruct
type option = Typs.mjOption cstruct
type voption = Typs.mjvOption cstruct
type rcontext = Typs.mjrContext cstruct
type vperturb = Typs.mjvPerturb cptr

type mouse = Typs.mjtMouse =
  | MjMOUSE_NONE
  | MjMOUSE_ROTATE_V
  | MjMOUSE_ROTATE_H
  | MjMOUSE_MOVE_V
  | MjMOUSE_MOVE_H
  | MjMOUSE_ZOOM
  | MjMOUSE_SELECT

let mouse_to_int mouse =
  Ctypes.(coerce Typs.mjtMouse uint32_t mouse) |> Unsigned.UInt32.to_int


type fontscale = Typs.mjtFontScale =
  | MjFONTSCALE_50
  | MjFONTSCALE_100
  | MjFONTSCALE_150
  | MjFONTSCALE_200
  | MjFONTSCALE_250
  | MjFONTSCALE_300

let fontscale_to_int fontscale =
  Ctypes.(coerce Typs.mjtFontScale uint32_t fontscale) |> Unsigned.UInt32.to_int


type catbit = Typs.mjtCatBit =
  | MjCAT_STATIC
  | MjCAT_DYNAMIC
  | MjCAT_DECOR
  | MjCAT_ALL

let catbit_to_int catbit =
  Ctypes.(coerce Typs.mjtCatBit uint32_t catbit) |> Unsigned.UInt32.to_int


type tstage = Typs.mjtStage =
  | MjSTAGE_NONE
  | MjSTAGE_POS
  | MjSTAGE_VEL
  | MjSTAGE_ACC

let tstage_to_int stage =
  Ctypes.(coerce Typs.mjtStage uint32_t stage) |> Unsigned.UInt32.to_int


type tsensor = Typs.mjtSensor =
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

let tsensor_to_int sensor =
  Ctypes.(coerce Typs.mjtSensor uint32_t sensor) |> Unsigned.UInt32.to_int


let load_xml ~name xml =
  Bindings.mj_loadXML xml Ctypes.(from_voidp Typs.mjVFS null) name 1000


let get_model_nv model = Ctypes.(getf !@model Typs.mjModel_nv)
let get_model_option model = Ctypes.(getf !@model Typs.mjModel_opt)
let delete_model = Bindings.mj_deleteModel
let forward = Bindings.mj_forward

let forward_skip model data stage sensor =
  Bindings.mj_forwardSkip model data (tstage_to_int stage) (tsensor_to_int sensor)


let inverse = Bindings.mj_inverse
let step = Bindings.mj_step
let step1 = Bindings.mj_step1
let step2 = Bindings.mj_step2
let make_data = Bindings.mj_makeData
let get_data_nefc data = Ctypes.(getf !@data Typs.mjData_nefc)
let get_data_qacc data = Ctypes.(getf !@data Typs.mjData_qacc)
let get_data_qfrc_inverse data = Ctypes.(getf !@data Typs.mjData_qfrc_inverse)
let reset_data = Bindings.mj_resetData
let delete_data = Bindings.mj_deleteData
let get_data_time data = Ctypes.(getf !@data Typs.mjData_time)
let get_option_iterations option = Ctypes.(getf option Typs.mjOption_iterations)
let get_option_tolerance option = Ctypes.(getf option Typs.mjOption_tolerance)

let set_option_iterations option iterations =
  Ctypes.(setf option Typs.mjOption_iterations iterations)


let set_option_tolerance option tolerance =
  Ctypes.(setf option Typs.mjOption_tolerance tolerance)


let make_default_vcamera () =
  let cam = Ctypes.make Typs.mjvCamera in
  Bindings.mjv_defaultCamera (Ctypes.addr cam);
  cam


let make_default_voption () =
  let opt = Ctypes.make Typs.mjvOption in
  Bindings.mjv_defaultOption (Ctypes.addr opt);
  opt


let make_default_vscene () =
  let scn = Ctypes.make Typs.mjvScene in
  Bindings.mjv_defaultScene (Ctypes.addr scn);
  scn


let make_default_rcontext () =
  let con = Ctypes.make Typs.mjrContext in
  Bindings.mjr_defaultContext (Ctypes.addr con);
  con


let move_camera model action dx dy scn cam =
  Bindings.mjv_moveCamera
    model
    (mouse_to_int action)
    dx
    dy
    Ctypes.(addr scn)
    Ctypes.(addr cam)


let make_rrect ~left ~width ~bottom ~height =
  let rect = Ctypes.make Typs.mjrRect in
  Ctypes.setf rect Typs.mjrRect_left left;
  Ctypes.setf rect Typs.mjrRect_width width;
  Ctypes.setf rect Typs.mjrRect_bottom bottom;
  Ctypes.setf rect Typs.mjrRect_height height;
  rect


let render viewport scn con =
  Bindings.mjr_render viewport Ctypes.(addr scn) Ctypes.(addr con)


let make_scene model scene = Bindings.mjv_makeScene model Ctypes.(addr scene)

let update_scene ?perturb model data opt cam catbit scn =
  let perturb =
    match perturb with
    | None         -> Ctypes.(from_voidp Typs.mjvPerturb null)
    | Some perturb -> perturb
  in
  Bindings.mjv_updateScene
    model
    data
    Ctypes.(addr opt)
    perturb
    (Ctypes.addr cam)
    (catbit_to_int catbit)
    Ctypes.(addr scn)


let make_context model context fontscale =
  Bindings.mjr_makeContext model Ctypes.(addr context) (fontscale_to_int fontscale)
