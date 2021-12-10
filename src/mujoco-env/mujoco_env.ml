open Mujoco

let model_file = Printf.sprintf "%s/../share/mujoco-env/%s" Findlib.(default_location ())

module type Env = sig
  type env
  type action
  type reward
  type observation

  val init : unit -> env
  val step : env -> action -> observation * reward * bool
  val reset : env -> observation
end

module Make (E : Env) = struct
  include E

  type timestep =
    { reward : reward
    ; action : action
    ; observation : observation
    }

  let simulate policy =
    let env = init () in
    let rec run terminated observation acc =
      if terminated
      then acc
      else (
        let action = policy observation in
        let observation, reward, terminated = step env action in
        let acc = { observation; reward; action } :: acc in
        run terminated observation acc)
    in
    run false (reset env) []
end

module PendulumE = struct
  type env =
    { model : Model.t
    ; data : Data.t
    ; qpos0 : Owl.Arr.arr
    ; qvel0 : Owl.Arr.arr
    }

  type observation = Owl.Arr.arr
  type action = Owl.Arr.arr
  type reward = float

  let observe env = Owl.Arr.concatenate [| env.data.qpos; env.data.qvel |] |> Owl.Arr.copy

  let reset env =
    Data.reset env.model env.data;
    observe env


  let init () =
    let model = Model.load_xml (model_file "inverted_pendulum.xml") in
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
    step env.model env.data;
    let o = observe env in
    let fs = Bigarray.Genarray.(get o [| 0 |]) in
    let not_terminated = Owl.Arr.not_inf o && Float.(abs fs <= 0.2) in
    o, reward, not not_terminated
end

module Pendulum = Make (PendulumE)
