module C = Configurator.V1

let () =
  let mujoco_dir =
    match Sys.getenv_opt "MUJOCO_DIR" with
    | Some x -> x
    | None   -> Printf.sprintf "%s/.mujoco/mujoco210" Unix.(getenv "HOME")
  in
  C.main ~name:"mujoco" (fun _ ->
      let libs =
        [ Printf.(sprintf "-L%s/bin" mujoco_dir)
        ; "-lmujoco210"
        ; "-lmujoco210nogl"
        ; "-lglfw.3"
        ]
      in
      let cflags = [ Printf.(sprintf "-I%s/include" mujoco_dir) ] in
      let conf : C.Pkg_config.package_conf = { cflags; libs } in
      C.Flags.write_sexp "c_flags.sexp" conf.cflags;
      C.Flags.write_sexp "c_library_flags.sexp" conf.libs;
      C.Flags.write_lines "c_flags" conf.cflags;
      C.Flags.write_lines "c_library_flags" conf.libs)
