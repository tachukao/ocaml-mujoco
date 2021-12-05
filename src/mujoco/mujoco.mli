open Bigarray
open Mujoco_core.Wrapper

type tensor = (float, float64_elt, c_layout) Genarray.t
type camera = mjvCamera
type option = mjvOption
type scene = mjvScene
type context = mjrContext

module Model : sig
  type t = { ptr : mjModel ptr }

  (** [load_xml path] load model from xml [path] *)
  val load_xml : string -> t

  (** [nv m] returns nv for model [m] *)
  val nv : t -> int

  (** [nu m] returns nu for model [m] *)
  val nu : t -> int
end

module Data : sig
  type t =
    { ptr : mjData ptr
    ; qpos : tensor
    ; qvel : tensor
    ; ctrl : tensor
    }

  (** [make m] make data with model [m] *)
  val make : Model.t -> t

  (** [reset m d] resets data [d] with model [m] *)
  val reset : Model.t -> t -> unit

  (** [time d] returns time for data [d] *)
  val time : t -> float
end

(** [step m d] step for model [m] and data [d] *)
val step : Model.t -> Data.t -> unit

(** [step1 m d] step1 for model [m] and data [d] *)
val step1 : Model.t -> Data.t -> unit

(** [step2 m d] step2 for model [m] and data [d] *)
val step2 : Model.t -> Data.t -> unit

(** [forward m d] forward for model [m] and data [d] *)
val forward : Model.t -> Data.t -> unit

module Viewport : sig
  type t = mjrRect

  (** [make ~left ~width ~bottom ~height] make viewport dimensions width and height, aligned left and bottom *)
  val make : left:int -> width:int -> bottom:int -> height:int -> t
end

(** [default_camera ()] returns default camera *)
val default_camera : unit -> camera

(** [default_option ()] returns default option *)
val default_option : unit -> option

(** [default_scene ()] returns default scene *)
val default_scene : unit -> scene

(** [default_scene ()] returns default context *)
val default_context : unit -> context

(** [visualise ~loop m d] visuzlise simulation [loop] for model [m] and data [d] *)
val visualise
  :  loop:(t0:float -> Model.t -> Data.t -> unit -> unit)
  -> Model.t
  -> Data.t
  -> unit

(** [record ?(width=1200) ?(height=900) ?(camera_scale=3.) ~duration ~fps ~advance m d file] 
    saves [width]x[height] video of simulation with [advance] function to [file] with 
    [camera_scale], [duration], [fps]. *)
val record
  :  ?width:int
  -> ?height:int
  -> ?camera_scale:float
  -> duration:float
  -> fps:int
  -> advance:(Model.t -> Data.t -> unit -> unit)
  -> Model.t
  -> Data.t
  -> string
  -> unit
