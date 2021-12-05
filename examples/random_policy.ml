(* Example of model controlled by a random policy *)
open Owl
open Mujoco

(* path to model xml *)
let model_xml = Cmdargs.(get_string "-xml" |> force ~usage:"model XML")

(* record video, must have ffmpeg installed  *)
let record_video = Cmdargs.(check "-record")

let advance model (data : Data.t) () =
  let nu = Model.nu model in
  Arr.(set_slice [ [ 0; -1 ] ] data.ctrl (0.1 $* tanh (gaussian [| nu |])));
  step model data


let loop ~t0 model data () =
  while Data.time data -. t0 < 30. /. 60. do
    advance model data ()
  done


let () =
  (* Create camera option scene and context *)
  let model = Model.load_xml model_xml ~name:"Example" in
  let data = Data.make model in
  if record_video
  then (
    let width = 640 in
    let height = 480 in
    let duration = 5. in
    let fps = 30 in
    record ~width ~height ~duration ~fps ~advance model data "video.mp4")
  else visualise ~loop model data
