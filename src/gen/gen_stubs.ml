let with_file ~f file =
  let oc = open_out file in
  let fmt = Format.formatter_of_out_channel oc in
  f fmt;
  close_out oc


let () =
  with_file "mujoco_stubs.c" ~f:(fun fmt_c ->
      Format.fprintf fmt_c "#include \"mujoco.h\"@.";
      Cstubs.write_c fmt_c ~prefix:"caml" (module Stubs.Bindings));
  with_file "mujoco_generated.ml" ~f:(fun fmt_ml ->
      Cstubs.write_ml fmt_ml ~prefix:"caml" (module Stubs.Bindings));
  flush_all ()
