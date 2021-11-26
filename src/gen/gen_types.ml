let with_file ~f file =
  let oc = open_out file in
  let fmt = Format.formatter_of_out_channel oc in
  f fmt;
  close_out oc


let () =
  with_file "mujoco_types.c" ~f:(fun fmt_c ->
      Format.fprintf fmt_c "#include \"mujoco.h\"@.";
      Cstubs.Types.write_c fmt_c (module Types.Bindings))
