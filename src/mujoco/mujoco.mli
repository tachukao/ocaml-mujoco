open Bigarray
open Wrapper

type tensor = (float, float64_elt, c_layout) Genarray.t
type camera = mjvCamera
type option = mjvOption
type scene = mjvScene
type context = mjrContext

module Model : sig
  type t = { ptr : mjModel ptr }

  val load_xml : name:string -> string -> t
  val nv : t -> int
  val nu : t -> int
end

module Data : sig
  type t =
    { ptr : mjData ptr
    ; qpos : tensor
    ; qvel : tensor
    ; ctrl : tensor
    }

  val reset : Model.t -> t -> unit
  val make : Model.t -> t
  val time : t -> float
end

val step : Model.t -> Data.t -> unit
val step1 : Model.t -> Data.t -> unit
val step2 : Model.t -> Data.t -> unit
val forward : Model.t -> Data.t -> unit

module Viewport : sig
  type t = mjrRect

  val make : left:int -> width:int -> bottom:int -> height:int -> t
end

val default_camera : unit -> camera
val default_option : unit -> option
val default_scene : unit -> scene
val default_context : unit -> context

val visualise
  :  loop:(t0:float -> Model.t -> Data.t -> unit -> unit)
  -> Model.t
  -> Data.t
  -> unit

val record
  :  ?width:int
  -> ?height:int
  -> ?camera_scale:float
  -> duration:float
  -> fps:float
  -> advance:(Model.t -> Data.t -> unit -> unit)
  -> Model.t
  -> Data.t
  -> string
  -> unit
