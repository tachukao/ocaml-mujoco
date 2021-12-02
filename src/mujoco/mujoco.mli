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

(** return int associated with mouse *)
val mouse_to_int : mouse -> int

type fontscale = Typs.mjtFontScale =
  | MjFONTSCALE_50
  | MjFONTSCALE_100
  | MjFONTSCALE_150
  | MjFONTSCALE_200
  | MjFONTSCALE_250
  | MjFONTSCALE_300

(** return int associated with fontscale *)
val fontscale_to_int : fontscale -> int

type catbit = Typs.mjtCatBit =
  | MjCAT_STATIC
  | MjCAT_DYNAMIC
  | MjCAT_DECOR
  | MjCAT_ALL

(** return int associated with catbit *)
val catbit_to_int : catbit -> int

type tstage = Typs.mjtStage =
  | MjSTAGE_NONE
  | MjSTAGE_POS
  | MjSTAGE_VEL
  | MjSTAGE_ACC

(** return int associated with tstage *)
val tstage_to_int : tstage -> int

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

(** return int associated with tsensor *)
val tsensor_to_int : tsensor -> int

(** load model from xml string with name *)
val load_xml : name:string -> string -> model

(** get model nv *)
val get_model_nv : model -> int

(** get model option *)
val get_model_option : model -> option

(** delete mujoco model *)
val delete_model : model -> unit

(** make data from model *)
val make_data : model -> data

(** get nefc from data *)
val get_data_nefc : data -> int

(** get qacc from data *)
val get_data_qacc : data -> float Ctypes_static.ptr

(** get qfrc_inverse from data *)
val get_data_qfrc_inverse : data -> float Ctypes_static.ptr

(** reset data *)
val reset_data : model -> data -> unit

(** delete data *)
val delete_data : data -> unit

(** get time of data *)
val get_data_time : data -> float

(** get iterations from option *)
val get_option_iterations : option -> int

(** get tolerance from option *)
val get_option_tolerance : option -> float

(** set iterations for option *)
val set_option_iterations : option -> int -> unit

(** set tolerance for option *)
val set_option_tolerance : option -> float -> unit

(** mj_forward *)
val forward : model -> data -> unit

(** mj_forwardSkip *)
val forward_skip : model -> data -> tstage -> tsensor -> unit

(** mj_inverse *)
val inverse : model -> data -> unit

(** mj_step *)
val step : model -> data -> unit

(** mj_step1 *)
val step1 : model -> data -> unit

(** mj_step2 *)
val step2 : model -> data -> unit

(** move camera *)
val move_camera : model -> mouse -> float -> float -> vscene -> vcamera -> unit

(** make rrect *)
val make_rrect : left:int -> width:int -> bottom:int -> height:int -> rrect

(** render on scene *)
val render : rrect -> vscene -> rcontext -> unit

(** make default vcamera *)
val make_default_vcamera : unit -> vcamera

(** make default voption *)
val make_default_voption : unit -> voption

(*** make default vscene *)
val make_default_vscene : unit -> vscene

(*** make default rcontext *)
val make_default_rcontext : unit -> rcontext

(** make scene *)
val make_scene : model -> vscene -> int -> unit

(** update scene *)
val update_scene
  :  ?perturb:vperturb
  -> model
  -> data
  -> voption
  -> vcamera
  -> catbit
  -> vscene
  -> unit

(** make context *)
val make_context : model -> rcontext -> fontscale -> unit
