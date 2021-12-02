(*
open Mujoco
open Wrapper

let model_xml = Cmdargs.(get_string "-xml" |> force ~usage:"model XML")
let niter = 30
let nwarmup = 3
let nepoch = 20
let nstep = 500
let eps = 1e-6

let worker model data =
  ignore model;
  ignore data


let () =
  Printf.printf "niter   : %d\n" niter;
  Printf.printf "nwarmup : %d\n" nwarmup;
  Printf.printf "nepoch  : %d\n" nepoch;
  Printf.printf "nstep   : %d\n" nstep;
  Printf.printf "eps     : %g\n\n" eps;
  let model =
    Bindings.mj_loadXML model_xml Ctypes.(from_voidp Typs.mjVFS null) "Example" 1000
  in
  let data = Bindings.mj_makeData model in
  let deriv =
    Unsigned.Size_t.(
      of_int
        (6
        * Ctypes.(sizeof double)
        * Ctypes.(getf !@model Typs.mjModel_nv)
        * Ctypes.(getf !@model Typs.mjModel_nv)))
    |> Bindings.mju_malloc
    |> Ctypes.from_voidp Ctypes.double
  in
  (* save solver options *)
  let save_iterations =
    Ctypes.(getf !@(model |-> Typs.mjModel_opt) Typs.mjOption_iterations)
  in
  let save_tolerance =
    Ctypes.(getf !@(model |-> Typs.mjModel_opt) Typs.mjOption_tolerance)
  in
  let nefc = ref 0 in
  for epoch = 0 to nepoch - 1 do
    (* set solver options for main simulation *)
    Ctypes.(setf !@(model |-> Typs.mjModel_opt) Typs.mjOption_iterations save_iterations);
    Ctypes.(setf !@(model |-> Typs.mjModel_opt) Typs.mjOption_tolerance save_tolerance);
    (* advance main simulation for nstep *)
    for i = 0 to nstep - 1 do
      Bindings.mj_step model data
    done;
    (nefc := Ctypes.(getf !@data Typs.mjData_nefc));
    (* set solver options for finite differences *)
    Ctypes.(setf !@(model |-> Typs.mjModel_opt) Typs.mjOption_iterations niter);
    Ctypes.(setf !@(model |-> Typs.mjModel_opt) Typs.mjOption_tolerance 0.);
    (* test forward and inverse *)
    for isforward = 0 to 1 do
      (* run worker *)
      worker model data
    done
  done;
  Bindings.mju_free Ctypes.(to_voidp deriv);
  Bindings.mj_deleteData data;
  Bindings.mj_deleteModel model
*)
