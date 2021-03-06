let fun_stubs_filename = "src/wrapper/stubs/stubs.ml"
let typs_stubs_filename = "src/wrapper/types/types.ml"
let wrapper_filename = "src/wrapper/wrapper.ml"

let () =
  Fun_gen.write fun_stubs_filename;
  Typs_gen.write typs_stubs_filename;
  Wrapper_gen.write wrapper_filename;
  Sys.command Printf.(sprintf "ocamlformat -i %s" fun_stubs_filename) |> ignore;
  Sys.command Printf.(sprintf "ocamlformat -i %s" typs_stubs_filename) |> ignore;
  Sys.command Printf.(sprintf "ocamlformat -i %s" wrapper_filename) |> ignore;
  Sys.command Printf.(sprintf "ocamlformat -i %si" wrapper_filename) |> ignore
