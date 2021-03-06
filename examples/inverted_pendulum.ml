open Mujoco
open Mujoco_env

let () =
  let env = Inverted_pendulum.init () in
  Owl.Arr.print Model.(ctrlrange env.model);
  let terminated = ref false in
  Printf.printf "start\n%!";
  while not !terminated do
    let action = Owl.Arr.uniform ~a:(-0.01) ~b:0.01 [| Model.nu env.model |] in
    let _, r, term = Inverted_pendulum.(step env action) in
    Printf.printf "%f\n%!" r;
    terminated := term
  done
