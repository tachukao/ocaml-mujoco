open Base

let file =
  let mujoco_dir =
    match Stdlib.Sys.getenv_opt "MUJOCO_DIR" with
    | Some x -> x
    | None   -> Printf.sprintf "%s/.mujoco/mujoco210" Unix.(getenv "HOME")
  in
  Printf.sprintf "%s/include/mujoco.h" mujoco_dir


let read_entire_file file = Stdio.In_channel.with_file file ~f:Stdio.In_channel.input_all

let convert_arg s =
  if String.(s = "void")
  then "void"
  else (
    (* remove const *)
    let s = Str.global_replace Str.(regexp "const ") "" s in
    (* get variable name *)
    let split = String.split ~on:' ' s in
    let vn = List.last_exn split in
    let typ =
      split
      |> List.drop_last_exn
      |> String.concat ~sep:" "
      |> fun x -> if String.contains vn '[' then Printf.sprintf "%s*" x else x
    in
    (* convert typ *)
    Common.convert_typ typ)


let parse s =
  (* use mjtMouse type instead of int *)
  let s = Str.global_replace Str.(regexp "int action") "mjtMouse action" s in
  let s = Str.global_replace Str.(regexp "int catmask") "mjtCatBit catmask" s in
  let open Re in
  let regex =
    Pcre.re "[ ]*MJAPI ([^ \n]+)[ ]+(mj[uvr]?_[^\\(]+)\\(([^;]+)\\);" |> compile
  in
  all regex s
  |> List.map ~f:(fun group ->
         let _rval = Group.get group 1 |> Common.convert_typ in
         let _fun = Group.get group 2 in
         let _args =
           Group.get group 3
           |> Str.global_replace (Str.regexp "[ \n]+") " "
           |> String.split ~on:','
           |> List.map ~f:convert_arg
           |> String.concat ~sep:" @-> "
         in
         Printf.(
           sprintf "let %s = foreign \"%s\" (%s @-> returning %s)" _fun _fun _args _rval))


let write_stubs parsed =
  let open Stdio.Out_channel in
  let f channel =
    fprintf
      channel
      "open Ctypes\n\
       module Typs = Typs\n\
       open Typs\n\n\
       module Bindings (F : FOREIGN) = struct\n\
       open F\n";
    List.iter ~f:(fprintf channel "%s\n") parsed;
    fprintf channel "end"
  in
  with_file "stubs.ml" ~f


let () = read_entire_file file |> parse |> write_stubs
