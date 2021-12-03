open Base
open Common

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
  ; cenum_states : string list
  }

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
  let regex = seq [ bol; shortest (group (rep1 wordc)); alt [ space; char ',' ] ] in
  all (compile regex) s |> List.map ~f:(fun group -> Group.get group 1)


(*
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
  *)

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


let is_nested_struct_fields s =
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
  Int.(List.length matched > 0)


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
         if is_nested_struct_fields _fields
         then { cstruct_name = name; cstruct_fields = Nested }
         else { cstruct_name = name; cstruct_fields = get_flat_struct_fields _fields })


let p_filename_box ch filename =
  let width = 80 in
  let l = String.length filename in
  let first_leg = 30 in
  let second_leg = width - (first_leg + 2) - l in
  p ch "(* %s *)" String.(make width '-');
  p ch "(* %s %s %s *)" String.(make first_leg '-') filename String.(make second_leg '-');
  p ch "(* %s *)\n\n" String.(make width '-')


let p_enum_part channel fs =
  List.iter (parse_enum fs) ~f:(fun cenum ->
      let name = cenum.cenum_name in
      let bname = get_bname name in
      let states = cenum.cenum_states in
      (* top *)
      p channel "\n%s" cenum.cenum_docstr;
      p channel "type %s =" bname;
      List.iter states ~f:(fun state -> p channel " | %s" String.(capitalize state));
      (* mid *)
      List.iter states ~f:(fun s -> p channel "let %s = constant \"%s\" int64_t" s s);
      (* bottom *)
      p channel "let %s =" bname;
      p channel " S.enum";
      p channel " \"%s\"" bname;
      p channel " ~typedef:true";
      p channel " [";
      List.iter states ~f:(fun s -> p channel "%s , %s;" String.(capitalize s) s);
      p channel " ]";
      p
        channel
        "    ~unexpected:(fun _ -> failwith \"unexpected %s element data type enum\")"
        bname)


let p_struct_part channel fs =
  List.iter (parse_struct fs) ~f:(fun cstruct ->
      let name = cstruct.cstruct_name in
      let bname = get_bname name in
      (* top *)
      p
        channel
        "type %s\nlet %s : %s structure typ = structure \"%s\"\n"
        name
        name
        name
        name;
      (* mid *)
      (match cstruct.cstruct_fields with
      | Nested      -> ()
      | Flat fields ->
        List.iter fields ~f:(fun (typ, field, docstr) ->
            p channel "%s" docstr;
            p channel "let %s_%s = field %s \"%s\" %s\n" bname field name field typ);
        (* seal *)
        p channel "let () = seal %s" name);
      (* bottom *)
      p channel "type %s = %s" bname name;
      p channel "let %s = %s" bname name)


let write_stubs ~stubs_filename s =
  Stdio.Out_channel.with_file stubs_filename ~f:(fun channel ->
      let ps = p channel in
      ps "(* THIS FILE IS GENERATED AUTOMATICALLY, DO NOT EDIT BY HAND *)\n";
      ps "open Ctypes";
      ps "module Bindings (S : Cstubs.Types.TYPE) = struct";
      ps "open S";
      ps "type mjtByte = Unsigned.UChar.t";
      ps "let mjtByte = uchar";
      ps "type mjtNum = float";
      ps "let mjtNum = double";
      ps "let mjfItemEnable = static_funptr (int @-> ptr void @-> returning int)\n";
      List.iter s ~f:(fun (filename, fs) ->
          p_filename_box channel filename;
          (* enum part *)
          p_enum_part channel fs;
          (* struct part *)
          p_struct_part channel fs);
      ps "end")


let write stubs_filename =
  [ "mjmodel.h"; "mjdata.h"; "mjvisualize.h"; "mjrender.h"; "mjui.h" ]
  |> List.map ~f:read_file
  |> write_stubs ~stubs_filename
