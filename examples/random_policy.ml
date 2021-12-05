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
    record
      ~width:1200
      ~height:900
      ~duration:3.
      ~fps:30.
      ~advance
      model
      data
      "recording.out";
    let command =
      Printf.(
        sprintf
          "ffmpeg -f rawvideo -pixel_format rgb24 -video_size %ix%i -framerate 30 -i  \
           recording.out -vf \"vflip\" video.mp4")
        1200
        900
    in
    Sys.command command |> ignore)
  else visualise ~loop model data
