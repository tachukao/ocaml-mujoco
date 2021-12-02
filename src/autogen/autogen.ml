let fun_stubs_filename = "src/stubs/stubs.ml"
let typs_stubs_filename = "src/stubs/types/types.ml"

let () =
  Fun_gen.write fun_stubs_filename;
  Typs_gen.write typs_stubs_filename;
  Sys.command Printf.(sprintf "ocamlformat -i %s" fun_stubs_filename) |> ignore;
  Sys.command Printf.(sprintf "ocamlformat -i %s" typs_stubs_filename) |> ignore;
