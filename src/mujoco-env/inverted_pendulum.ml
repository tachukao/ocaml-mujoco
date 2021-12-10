open Mujoco

module E = struct
  type env =
    { model : Model.t
    ; data : Data.t
    ; qpos0 : Owl.Arr.arr
    ; qvel0 : Owl.Arr.arr
    }

  type observation = Owl.Arr.arr
  type action = Owl.Arr.arr
  type reward = float

  let frame_skip = 2
  let observe env = Owl.Arr.concatenate [| env.data.qpos; env.data.qvel |] |> Owl.Arr.copy

  let reset env =
    Data.reset env.model env.data;
    observe env


  let init () =
    let model = Model.load_xml (Env.model_file "inverted_pendulum.xml") in
    let data = Data.make model in
    let qpos0 = Owl.Arr.copy data.qpos in
    let qvel0 = Owl.Arr.copy data.qpos in
    let qpos =
      let dpos = Owl.Arr.uniform ~a:(-0.01) ~b:0.01 Owl.Arr.(shape qpos0) in
      Owl.Arr.(dpos + qpos0)
    in
    let qvel =
      let dvel = Owl.Arr.uniform ~a:(-0.01) ~b:0.01 Owl.Arr.(shape qvel0) in
      Owl.Arr.(dvel + qvel0)
    in
    Bigarray.Genarray.blit qpos data.qpos;
    Bigarray.Genarray.blit qvel data.qvel;
    { model; data; qpos0; qvel0 }


  let step env action =
    let reward = 1. in
    Bigarray.Genarray.blit action env.data.ctrl;
    for _ = 0 to frame_skip - 1 do
      step env.model env.data
    done;
    let o = observe env in
    let fs = Bigarray.Genarray.(get o [| 0 |]) in
    let not_terminated = Owl.Arr.not_inf o && Float.(abs fs <= 0.2) in
    o, reward, not not_terminated
end

include Env.Make (E)
