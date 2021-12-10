let model_file = Printf.sprintf "%s/../share/mujoco-env/%s" Findlib.(default_location ())

module type E = sig
  type env
  type action
  type reward
  type observation

  val init : unit -> env
  val step : env -> action -> observation * reward * bool
  val reset : env -> observation
end

module Make (E : E) = struct
  include E

  type timestep =
    { reward : reward
    ; action : action
    ; observation : observation
    }

  let loop update =
    let env = init () in
    let rec run terminated observation acc =
      if terminated
      then acc
      else (
        let terminated, observation, acc = update env observation acc in
        run terminated observation acc)
    in
    run false (reset env) []


  let simulate policy =
    let update env observation acc =
      let action = policy observation in
      let observation, reward, terminated = step env action in
      let acc = { observation; reward; action } :: acc in
      terminated, observation, acc
    in
    loop update
end
