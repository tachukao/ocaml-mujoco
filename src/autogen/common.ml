open Base

let convert_docstr s =
  let s = s |> String.strip |> Str.global_replace Str.(regexp "//") "" in
  "(** " ^ s ^ " *)"


let read_file filename =
  let mujoco_dir =
    match Stdlib.Sys.getenv_opt "MUJOCO_DIR" with
    | Some x -> x
    | None   -> Printf.sprintf "%s/.mujoco/mujoco210" Unix.(getenv "HOME")
  in
  let s =
    Printf.sprintf "%s/include/%s" mujoco_dir filename
    |> Stdio.In_channel.with_file ~f:Stdio.In_channel.input_all
  in
  filename, s


let p out_channel s =
  Printf.ksprintf
    (fun line ->
      Stdio.Out_channel.output_string out_channel line;
      Stdio.Out_channel.output_char out_channel '\n')
    s


let to_ctypes x =
  match String.(strip x) with
  | "unsigned char" -> "uchar"
  | x               -> x


let convert_typ s =
  if String.(strip s = "char*")
  then "string"
  else (
    match String.split ~on:'*' s with
    | []       -> failwith "Split string should not reach here"
    | [ hd ]   -> to_ctypes hd
    | hd :: tl ->
      let n = List.length tl in
      let left = List.init n ~f:(fun _ -> "(ptr ") |> String.concat ~sep:"" in
      let right = List.init n ~f:(fun _ -> ")") |> String.concat ~sep:"" in
      Printf.sprintf "%s%s%s" left (to_ctypes hd) right)
