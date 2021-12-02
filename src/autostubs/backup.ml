open Base

let file =
  let mujoco_dir =
    match Stdlib.Sys.getenv_opt "MUJOCO_DIR" with
    | Some x -> x
    | None   -> Printf.sprintf "%s/.mujoco/mujoco210" Unix.(getenv "HOME")
  in
  Printf.sprintf "%s/include/mujoco.h" mujoco_dir


let read_entire_file file = Stdio.In_channel.with_file file ~f:Stdio.In_channel.input_all

let convert_typ s =
  match String.split ~on:'*' s with
  | []       -> failwith "Split string should not reach here"
  | [ hd ]   -> hd
  | hd :: tl ->
    let n = List.length tl in
    let left = List.init n ~f:(fun _ -> "(ptr ") |> String.concat ~sep:"" in
    let right = List.init n ~f:(fun _ -> ")") |> String.concat ~sep:"" in
    Printf.sprintf "%s%s%s" left hd right


let convert_arg s =
  if String.(s = "void")
  then "void"
  else
    (* remove const *)
    Str.global_replace Str.(regexp "const ") "" s
    (* remove variable name *)
    |> String.split ~on:' '
    |> List.drop_last_exn
    |> String.concat ~sep:" "
    (* convert typ *)
    |> convert_typ


let parse s =
  let regex =
    Str.regexp "[ ]*MJAPI \\([^ \n]+\\)[ ]+\\(mj[uvr]?_[^(]+\\)(\\([^;]+\\));"
  in
  let rec append ofs acc =
    try
      let ofs = Str.search_forward regex s ofs in
      (* entire regular expression *)
      let _s = Str.matched_group 0 s in
      let ofs = ofs + String.length _s in
      let _rval = Str.matched_group 1 s |> convert_typ in
      let _fun = Str.matched_group 2 s in
      let _args =
        Str.matched_group 3 s
        |> Str.global_replace (Str.regexp "[ \n]+") " "
        |> String.split ~on:','
        |> List.map ~f:convert_arg
        |> String.concat ~sep:" @-> "
      in
      let next =
        Printf.(
          sprintf "let %s = foreign \"%s\" (%s @-> returning %s)" _fun _fun _args _rval)
      in
      append ofs (next :: acc)
    with
    | Stdlib.Not_found -> List.rev acc
  in
  append 0 []


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
  with_file "auto_stubs.ml" ~f


let () = read_entire_file file |> parse |> write_stubs
