open Base
open Common

let to_ocaml_type =
  let struct_names =
    List.map mujoco_type_file_names ~f:(fun filename ->
        let _, file = read_file filename in
        let parsed_struct = parse_struct file in
        List.map ~f:(fun cstruct -> get_bname cstruct.cstruct_name) parsed_struct)
    |> Stdlib.List.flatten
  in
  fun s ->
    let ptr = Printf.sprintf "(ptr %s)" in
    let ptr2 = Printf.sprintf "%s ptr" in
    let convert_structs s =
      match List.find struct_names ~f:(fun x -> String.(ptr x = s)) with
      | None   -> s
      | Some x -> ptr2 x
    in
    match s with
    | "uchar"            -> "Unsigned.UChar.t"
    | "size_t"           -> "Unsigned.Size_t.t"
    | "(ptr uchar)"      -> "Unsigned.UChar.t ptr"
    | "mjtNum"           -> "float"
    | "mjtByte"          -> "Unsigned.UChar.t"
    | "void"             -> "unit"
    | "double"           -> "float"
    | "(ptr int)"        -> "int ptr"
    | "(ptr mjtNum)"     -> "float ptr"
    | "(ptr mjtByte)"    -> "Unsigned.UChar.t ptr"
    | "(ptr float)"      -> "float ptr"
    | "(ptr double)"     -> "float ptr"
    | "(ptr void)"       -> "unit ptr"
    | "(ptr (ptr void))" -> "unit ptr ptr"
    | x                  -> convert_structs x


let p_types ch chi =
  List.map mujoco_type_file_names ~f:(fun filename ->
      let _, file = read_file filename in
      (* enum types *)
      let enum_callback =
        List.map (parse_enum file) ~f:(fun cenum ->
            let name = cenum.cenum_name in
            let bname = get_bname name in
            let states = cenum.cenum_states in
            p ch "\n";
            p chi "\n";
            (* top *)
            if String.(strip cenum.cenum_docstr = "(**  *)")
            then (
              p ch "(** %s *)" bname;
              p chi "(** %s *)" bname)
            else (
              p ch "%s" cenum.cenum_docstr;
              p chi "%s" cenum.cenum_docstr);
            p ch "type %s = Typs.%s =" bname bname;
            p chi "type %s = Typs.%s =" bname bname;
            List.iter states ~f:(fun (state, doc) ->
                p ch " | %s %s" String.(capitalize state) doc;
                p chi " | %s %s" String.(capitalize state) doc);
            let callback () =
              p ch "\n";
              p chi "\n";
              (* type to int *)
              p ch "(** convert %s type to int *)" bname;
              p chi "(** convert %s type to int *)" bname;
              p ch "let %s_to_int %s =" bname bname;
              p
                ch
                "Ctypes.(coerce Typs.%s uint32_t %s) |> Unsigned.UInt32.to_int"
                bname
                bname;
              p chi "val %s_to_int : %s -> int" bname bname
            in
            callback)
      in
      let parsed_struct = parse_struct file in
      let struct_callback =
        (* struct types *)
        List.map parsed_struct ~f:(fun cstruct ->
            let name = cstruct.cstruct_name in
            let bname = get_bname name in
            let cstruct_fields = cstruct.cstruct_fields in
            (* define types *)
            p ch "\n";
            p chi "\n";
            p ch "(** type %s *)" bname;
            p chi "(** type %s *)" bname;
            p ch "type %s = Typs.%s t" bname bname;
            p chi "type %s = Typs.%s t" bname bname;
            (* define set and get functions *)
            let callback () =
              match cstruct_fields with
              | Nested      -> ()
              | Flat fields ->
                (* allocate fresh *)
                p ch "\n";
                p chi "\n";
                p ch "(** allocate fresh %s struct *)" bname;
                p chi "(** allocate fresh %s struct *)" bname;
                p ch "let %s_allocate () = Ctypes.(make Typs.%s)" bname bname;
                p chi "val %s_allocate : unit -> %s" bname bname;
                (* make null ptr *)
                p ch "\n";
                p chi "\n";
                p ch "(** make null %s struct ptr *)" bname;
                p chi "(** make null %s struct ptr *)" bname;
                p ch "let %s_null () = Ctypes.(from_voidp Typs.%s null)" bname bname;
                p chi "val %s_null : unit -> %s ptr" bname bname;
                List.iter fields ~f:(fun (typ, field, _) ->
                    let typ = to_ocaml_type typ in
                    (* getter *)
                    p ch "\n";
                    p chi "\n";
                    p ch "(** get %s from %s *)" field bname;
                    p chi "(** get %s from %s *)" field bname;
                    p ch "let %s_get_%s x =" bname field;
                    p ch "  Ctypes.(getf x Typs.%s_%s)" bname field;
                    p chi "val %s_get_%s : %s -> %s" bname field bname typ;
                    (* setter *)
                    p ch "\n";
                    p chi "\n";
                    p ch "(** set %s for %s *)" field bname;
                    p chi "(** set %s for %s *)" field bname;
                    p ch "let %s_set_%s x y =" bname field;
                    p ch "  Ctypes.(setf x Typs.%s_%s y)" bname field;
                    p chi "val %s_set_%s : %s -> %s -> unit" bname field bname typ)
            in
            callback)
      in
      Caml.List.flatten [ enum_callback; struct_callback ])
  |> Caml.List.flatten


let p_func ch chi =
  let sections = parse_sect (snd (read_file "mujoco.h")) in
  List.iter sections ~f:(fun section ->
      match section with
      | `Delim delim -> p ch "%s\n" (convert_docstr Re.Group.(get delim 1))
      | `Text s      ->
        List.iter (parse_func s) ~f:(fun cfunc ->
            p ch "\n";
            p ch "%s" cfunc.docstr;
            p chi "\n";
            p chi "%s" cfunc.docstr;
            p ch "let %s = Bindings.%s" cfunc.func cfunc.func;
            p
              chi
              "val %s : %s -> %s"
              cfunc.func
              String.(concat ~sep:" -> " List.(map ~f:to_ocaml_type cfunc.args))
              (to_ocaml_type cfunc.rval);
            p ch "\n"))


let write_wrapper ~filename =
  Stdio.Out_channel.with_file (filename ^ "i") ~f:(fun chi ->
      Stdio.Out_channel.with_file filename ~f:(fun ch ->
          p_auto ch;
          p_auto chi;
          let pc = p ch in
          let pci = p chi in
          pc "module Typs = Stubs.Typs";
          pc "module Bindings = Stubs.Bindings (Mujoco_generated)";
          pci "open Stubs";
          (* pci "module Typs : module type of Typs"; *)
          (* pci "module Bindings : module type of Stubs.Bindings (Mujoco_generated)"; *)
          pc "type 'a t = 'a Ctypes.structure";
          pci "type 'a t = 'a Ctypes.structure";
          pc "type 'a ptr = 'a Ctypes_static.ptr";
          pci "type 'a ptr = 'a Ctypes_static.ptr";
          pc "let ( !@ ) = Ctypes.( !@ )";
          pci "val ( !@ ) : 'a ptr -> 'a";
          pc "let ( !& ) = Ctypes.addr";
          pci "val ( !& ): 'a t -> 'a t ptr";
          pc
            "type mjfItemEnable = (int -> unit ptr -> int) \
             Mujoco_generated.CI.static_funptr";
          pci
            "type mjfItemEnable = (int -> unit ptr -> int) \
             Mujoco_generated.CI.static_funptr";
          let callbacks = p_types ch chi in
          List.iter callbacks ~f:(fun g -> g ());
          p_func ch chi))


let write filename = write_wrapper ~filename
