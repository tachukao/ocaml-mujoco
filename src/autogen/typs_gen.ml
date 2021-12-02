open Base

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


let get_bname name =
  assert (Char.(name.[0] = '_'));
  String.sub ~pos:1 ~len:String.(length name - 1) name


let convert_enum name items =
  let bname = get_bname name in
  let top =
    items
    |> List.map ~f:String.capitalize
    |> List.map ~f:Printf.(sprintf "| %s")
    |> String.concat ~sep:"\n"
    |> Printf.sprintf "type %s =\n%s\n" bname
  in
  let mid =
    items
    |> List.map ~f:(fun s -> Printf.(sprintf "let %s = constant \"%s\" int64_t" s s))
    |> String.concat ~sep:"\n"
  in
  let bottom =
    let z =
      items
      |> List.map ~f:(fun s -> Printf.sprintf "%s , %s" String.(capitalize s) s)
      |> String.concat ~sep:"\n; "
      |> Printf.sprintf "[ %s ]"
    in
    Printf.sprintf
      "let %s =\n\
       S.enum\n\
       \"%s\"\n\
       ~typedef:true\n\
       %s\n\
      \      ~unexpected:(fun _ -> failwith \"unexpected %s element data type enum\")\n"
      bname
      bname
      z
      bname
  in
  Printf.sprintf "%s\n%s\n%s\n%!" top mid bottom


let parse_enum s =
  let open Re in
  let get_items s =
    let s =
      String.strip s
      |> Printf.sprintf "%s,"
      |> String.split_lines
      |> List.map ~f:String.strip
      |> List.filter ~f:(fun s -> String.(s <> ""))
      |> String.concat ~sep:"\n"
    in
    let regex = seq [ bol; shortest (group (rep1 wordc)); alt [ space; char ',' ] ] in
    all (compile regex) s |> List.map ~f:(fun group -> Group.get group 1)
  in
  let regex =
    seq
      [ bol
      ; seq [ bow; str "typedef enum"; eow ]
      ; space
      ; group (rep wordc)
      ; shortest (rep any)
      ; str "{"
      ; shortest (rep notnl)
      ; eol
      ; group (shortest (rep any))
      ; str "}"
      ]
  in
  all (compile regex) s
  |> List.map ~f:(fun group ->
         let name = Group.get group 1 in
         let items = Group.get group 2 |> get_items in
         convert_enum name items)
  |> String.concat ~sep:"\n"


let convert_struct name fields =
  let bname = get_bname name in
  let top =
    Printf.sprintf
      "type %s\nlet %s : %s structure typ = structure \"%s\""
      name
      name
      name
      name
  in
  let seal = Printf.sprintf "let () = seal %s" name in
  let bottom = Printf.sprintf "type %s = %s\nlet %s = %s" bname name bname name in
  let open Re in
  let get_fields s =
    let regex =
      seq
        [ bol
        ; rep blank
        ; shortest (group (rep (alt [ wordc; char '*' ])))
        ; rep blank
        ; shortest (group (rep wordc))
        ; str ";"
        ]
    in
    let mid =
      all (compile regex) s
      |> List.map ~f:(fun group ->
             let typ = Group.get group 1 |> Common.convert_typ in
             let field = Group.get group 2 in
             Printf.sprintf "let %s_%s = field %s \"%s\" %s" bname field name field typ)
      |> String.concat ~sep:"\n"
    in
    Printf.sprintf "%s\n%s\n" mid seal
  in
  let regex =
    seq
      [ seq [ bow; str "struct"; eow ]
      ; shortest (rep any)
      ; str "{"
      ; shortest (group (rep any))
      ; seq [ str "}"; space; group (rep wordc) ]
      ]
  in
  let matched = all (compile regex) fields in
  let mid = if not Int.(List.length matched = 0) then "" else get_fields fields in
  Printf.sprintf "%s\n%s\n%s\n" top mid bottom


let parse_struct s =
  let open Re in
  let regex =
    seq
      [ bol
      ; seq [ bow; str "struct"; eow ]
      ; space
      ; group (rep wordc)
      ; shortest (rep any)
      ; str "{"
      ; shortest (group (rep any))
      ; seq [ bol; str "};" ]
      ]
  in
  all (compile regex) s
  |> List.map ~f:(fun group ->
         let name = Group.get group 1 in
         let fields = Group.get group 2 in
         convert_struct name fields)
  |> String.concat ~sep:"\n\n"


let write_stubs ~stubs_filename parsed_list =
  let open Stdio.Out_channel in
  let f channel =
    fprintf
      channel
      "(* THIS FILE IS GENERATED AUTOMATICALLY, DO NOT EDIT BY HAND *)\n\n\
       open Ctypes\n\
       module Bindings (S : Cstubs.Types.TYPE) = struct\n\
       open S\n\
       type mjtByte = Unsigned.UChar.t\n\
       let mjtByte = uchar\n\
       type mjtNum = float\n\
       let mjtNum = double\n\
       let mjfItemEnable = static_funptr (int @-> ptr void @-> returning int)\n\n";
    List.iter
      ~f:(fun (filename, parsed) ->
        fprintf
          channel
          "(* %s %s %s *)\n\n%s\n"
          String.(make 25 '-')
          filename
          String.(make 25 '-')
          parsed)
      parsed_list;
    fprintf channel "end"
  in
  with_file stubs_filename ~f


let parse s =
  let _enum = parse_enum s in
  let _struct = parse_struct s in
  Printf.sprintf "%s\n%s" _enum _struct


let write stubs_filename =
  [ "mjmodel.h"; "mjdata.h"; "mjvisualize.h"; "mjrender.h"; "mjui.h" ]
  |> List.map ~f:read_file
  |> List.map ~f:(fun (n, x) -> n, parse x)
  |> write_stubs ~stubs_filename
