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

  let frame_skip = 5
  let dt (model : Model.t) = Model.(timestep model) *. Int.(to_float frame_skip)
  let ctrl_force_min = -1.
  let ctrl_force_max = -1.
  let healthy_z_min = 0.2
  let healthy_z_max = 1.
  let terminate_when_unhealthy = true
  let healthy_reward = 1.
  let contact_cost_weight = 5e-4
  let ctrl_cost_weight = 0.5
  let reset_noise_scale = 0.1
  let exclude_current_positions_from_observation = true

  let contact_forces env =
    let raw_contact_forces = env.data.cfrc_ext in
    Owl.Arr.clip_by_value ~amin:ctrl_force_min ~amax:ctrl_force_max raw_contact_forces


  let control_cost action = ctrl_cost_weight *. Owl.Arr.(l2norm_sqr' action)
  let contact_cost env = contact_cost_weight *. Owl.Arr.(l2norm_sqr' (contact_forces env))

  let is_healthy env =
    let state = Env.state_vector env.data in
    let z = Bigarray.Genarray.get state [| 2 |] in
    Owl.Arr.not_inf state && healthy_z_min <= z && z <= healthy_z_max


  let is_terminated env = if terminate_when_unhealthy then not (is_healthy env) else false

  let healthy_reward env =
    if is_healthy env || terminate_when_unhealthy then healthy_reward else 0.


  let observe env =
    let cf = contact_forces env in
    let qpos =
      if exclude_current_positions_from_observation
      then Owl.Arr.get_slice [ [ 2; -1 ] ] env.data.qpos
      else env.data.qpos
    in
    Owl.Arr.concatenate [| qpos; env.data.qvel; Owl.Arr.flatten cf |] |> Owl.Arr.copy


  let reset env =
    let noise_low = -.reset_noise_scale in
    let noise_high = reset_noise_scale in
    let qpos =
      let dpos = Owl.Arr.uniform ~a:noise_low ~b:noise_high Owl.Arr.(shape env.qpos0) in
      Owl.Arr.(dpos + env.qpos0)
    in
    let qvel =
      let dvel = Owl.Arr.gaussian Owl.Arr.(shape env.qvel0) in
      Owl.Arr.(dvel + (reset_noise_scale $* env.qvel0))
    in
    Bigarray.Genarray.blit qpos env.data.qpos;
    Bigarray.Genarray.blit qvel env.data.qvel;
    observe env


  let init () =
    let model = Model.load_xml (Env.model_file "ant.xml") in
    let data = Data.make model in
    let qpos0 = Owl.Arr.copy data.qpos in
    let qvel0 = Owl.Arr.copy data.qpos in
    let env = { model; data; qpos0; qvel0 } in
    reset env |> ignore;
    env


  let xy_position data =
    Data.get_body_xpos ~name:"torso" data
    |> Owl.Arr.get_slice [ []; [ 0; 1 ] ]
    |> Owl.Arr.flatten
    |> Owl.Arr.copy


  let step env action =
    let xy_position_before = xy_position env.data in
    Bigarray.Genarray.blit action env.data.ctrl;
    for _ = 0 to frame_skip - 1 do
      step env.model env.data
    done;
    let xy_position_after = xy_position env.data in
    let xy_velocity =
      Owl.Arr.((xy_position_after - xy_position_before) /$ dt env.model)
    in
    let x_vel = Owl.Arr.get xy_velocity [| 0 |] in
    let ctrl_cost = control_cost action in
    let contact_cost = contact_cost env in
    let forward_reward = x_vel in
    let healthy_reward = healthy_reward env in
    let reward = forward_reward +. healthy_reward -. ctrl_cost -. contact_cost in
    let terminated = is_terminated env in
    let o = observe env in
    o, reward, terminated
end

include Env.Make (E)
