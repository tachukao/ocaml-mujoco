open Base
open Common

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
      p channel "\n";
      if String.(strip cenum.cenum_docstr = "(**  *)")
      then p channel "(** %s *)" bname
      else p channel "%s" cenum.cenum_docstr;
      p channel "type %s =" bname;
      List.iter states ~f:(fun (state, doc) ->
          p channel " | %s %s" String.(capitalize state) doc);
      (* mid *)
      List.iter states ~f:(fun (s, _) -> p channel "let %s = constant \"%s\" int64_t" s s);
      (* bottom *)
      p channel "let %s =" bname;
      p channel " S.enum";
      p channel " \"%s\"" bname;
      p channel " ~typedef:true";
      p channel " [";
      List.iter states ~f:(fun (s, _) -> p channel "%s , %s;" String.(capitalize s) s);
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
      p_auto channel;
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
  mujoco_type_file_names |> List.map ~f:read_file |> write_stubs ~stubs_filename
