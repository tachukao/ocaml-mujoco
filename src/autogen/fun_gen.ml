open Base
open Common

let write_stubs ~stubs_filename s =
  let sections = parse_sect s in
  Stdio.Out_channel.with_file stubs_filename ~f:(fun ch ->
      let ps = p ch in
      p_auto ch;
      ps "open Ctypes";
      ps "module Typs = Typs";
      ps "open Typs";
      ps "module Bindings (F : FOREIGN) = struct";
      ps "open F\n";
      List.iter sections ~f:(fun section ->
          match section with
          | `Delim delim -> p ch "%s\n" (convert_docstr Re.Group.(get delim 1))
          | `Text s      ->
            List.iter (parse_func s) ~f:(fun cfunc ->
                p ch "%s" cfunc.docstr;
                p
                  ch
                  "let %s = foreign \"%s\" (%s @-> returning %s)"
                  cfunc.func
                  cfunc.func
                  String.(concat ~sep:" @-> " cfunc.args)
                  cfunc.rval;
                p ch "\n"));
      ps "end")


let write stubs_filename = snd (read_file "mujoco.h") |> write_stubs ~stubs_filename
