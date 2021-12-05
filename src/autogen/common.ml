open Base

let mujoco_type_file_names =
  [ "mjmodel.h"; "mjdata.h"; "mjvisualize.h"; "mjrender.h"; "mjui.h" ]


type cstruct_fields =
  | Nested
  | Flat of (string * string * string) list

type cstruct =
  { cstruct_name : string
  ; cstruct_fields : cstruct_fields
  }

type cenum =
  { cenum_name : string
  ; cenum_docstr : string
  ; cenum_states : (string * string) list
  }

type cfunc =
  { docstr : string
  ; rval : string
  ; args : string list
  ; func : string
  }

let convert_docstr s =
  let s = s |> String.strip |> Str.global_replace Str.(regexp "//") "" |> String.strip in
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


let p_auto ch = p ch "(* THIS FILE IS GENERATED AUTOMATICALLY, DO NOT EDIT BY HAND *)\n"

let to_ctypes x =
  match String.(strip x) with
  | "unsigned char" -> "uchar"
  | x               -> x


let convert_typ s =
  match String.split ~on:'*' s with
  | []       -> failwith "Split string should not reach here"
  | [ hd ]   -> to_ctypes hd
  | hd :: tl ->
    let n = List.length tl in
    let left = List.init n ~f:(fun _ -> "(ptr ") |> String.concat ~sep:"" in
    let right = List.init n ~f:(fun _ -> ")") |> String.concat ~sep:"" in
    Printf.sprintf "%s%s%s" left (to_ctypes hd) right


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
      |> fun x -> if String.contains vn '[' then x ^ "*" else x
    in
    (* convert typ *)
    convert_typ typ)


let parse_sect s =
  let open Re in
  let regex = seq [ bol; group (seq [ str "//-"; rep notnl ]); str "\n" ] in
  split_full (compile regex) s


let parse_func s =
  let open Re in
  let regex =
    seq
      [ group (rep (seq [ str "//"; rep notnl; str "\n" ]))
      ; bol
      ; seq [ bow; str "MJAPI"; eow ]
      ; rep blank
      ; group (rep (alt [ wordc; char '*' ])) (* type *)
      ; rep blank
      ; group
          (seq
             [ str "mj"
             ; opt (alt [ char 'u'; char 'v'; char 'r'; str "ui" ])
             ; char '_'
             ; rep wordc
             ])
      ; char '('
      ; group (shortest (rep any))
      ; char ')'
      ]
  in
  all (compile regex) s
  |> List.map ~f:(fun group ->
         let docstr = Group.(get group 1) |> convert_docstr in
         (* Stdio.printf "%s\n%!" docstr; *)
         let rval = Group.get group 2 |> convert_typ in
         let func = Group.get group 3 in
         let args =
           Group.get group 4
           |> Str.global_replace (Str.regexp "[ \n]+") " "
           |> String.split ~on:','
           |> List.map ~f:convert_arg
         in
         { docstr; rval; func; args })


let get_bname name =
  assert (Char.(name.[0] = '_'));
  String.sub ~pos:1 ~len:String.(length name - 1) name


let get_enum_states s =
  let open Re in
  let s =
    String.strip s
    |> Printf.sprintf "%s,"
    |> String.split_lines
    |> List.map ~f:String.strip
    |> List.filter ~f:(fun s -> String.(s <> ""))
    |> String.concat ~sep:"\n"
  in
  let regex =
    seq
      [ bol
      ; shortest (group (rep1 wordc))
      ; alt [ space; char ',' ]
      ; shortest (rep notnl)
      ; char '/'
      ; group (rep notnl)
      ]
  in
  all (compile regex) s
  |> List.map ~f:(fun group ->
         let s = Group.get group 1 in
         let docstr = convert_docstr ("/" ^ Group.(get group 2)) in
         s, docstr)


let parse_enum s =
  let open Re in
  let regex =
    seq
      [ bol
      ; seq [ bow; str "typedef enum"; eow ]
      ; space
      ; group (rep wordc)
      ; group (shortest (rep any))
      ; str "{"
      ; shortest (rep notnl)
      ; eol
      ; group (shortest (rep any))
      ; str "}"
      ]
  in
  all (compile regex) s
  |> List.map ~f:(fun group ->
         let cenum_name = Group.get group 1 in
         let cenum_docstr = Group.get group 2 |> convert_docstr in
         let cenum_states = Group.get group 3 |> get_enum_states in
         { cenum_docstr; cenum_name; cenum_states })


let is_nested_struct_fields name s =
  let open Re in
  let regex =
    seq
      [ seq [ bow; str "struct"; eow ]
      ; shortest (rep any)
      ; str "{"
      ; shortest (group (rep any))
      ; seq [ str "}"; space; group (rep wordc) ]
      ]
  in
  let matched = all (compile regex) s in
  let b = Int.(List.length matched > 0) || String.(name = "_mjuiItem") in
  if b then Stdio.printf "%s\n%!" name;
  b


let get_flat_struct_fields s =
  let open Re in
  let regex =
    seq
      [ bol
      ; rep blank
      ; shortest (group (rep (alt [ wordc; char '*' ])))
      ; rep blank
      ; shortest (group (rep (alt [ wordc ])))
      ; opt (group (seq [ char '['; rep wordc; char ']' ]))
      ; str ";"
      ; group (rep notnl)
      ]
  in
  let fields =
    all (compile regex) s
    |> List.map ~f:(fun group ->
           let typ =
             let typ = Group.get group 1 in
             (match Group.(get_opt group 3) with
             | None   -> typ
             | Some _ -> typ ^ "*")
             |> convert_typ
           in
           let field = Group.get group 2 in
           let docstr = Group.(get group 4) |> convert_docstr in
           typ, field, docstr)
  in
  Flat fields


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
         let _fields = Group.get group 2 in
         if is_nested_struct_fields name _fields
         then { cstruct_name = name; cstruct_fields = Nested }
         else { cstruct_name = name; cstruct_fields = get_flat_struct_fields _fields })
