open Base

let p = Common.p

type cfunc =
  { docstr : string
  ; rval : string
  ; args : string list
  ; func : string
  }

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
    Common.convert_typ typ)


let convert_docstr s =
  let s = s |> String.strip |> Str.global_replace Str.(regexp "//") "" in
  "(** " ^ s ^ " *)"


let parse s =
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
             [ str "mj"; opt (alt [ char 'u'; char 'v'; char 'r' ]); char '_'; rep wordc ])
      ; char '('
      ; group (shortest (rep any))
      ; char ')'
      ]
  in
  all (compile regex) s
  |> List.map ~f:(fun group ->
         let docstr = Group.(get group 1) |> convert_docstr in
         (* Stdio.printf "%s\n%!" docstr; *)
         let rval = Group.get group 2 |> Common.convert_typ in
         let func = Group.get group 3 in
         let args =
           Group.get group 4
           |> Str.global_replace (Str.regexp "[ \n]+") " "
           |> String.split ~on:','
           |> List.map ~f:convert_arg
         in
         { docstr; rval; func; args })


let write_stubs ~stubs_filename parsed =
  let f ch =
    let ps = p ch in
    ps "(* THIS FILE IS GENERATED AUTOMATICALLY, DO NOT EDIT BY HAND *)\n";
    ps "open Ctypes";
    ps "module Typs = Typs";
    ps "open Typs";
    ps "module Bindings (F : FOREIGN) = struct";
    ps "open F\n";
    List.iter parsed ~f:(fun { docstr; func; args; rval } ->
        p ch "%s" docstr;
        p
          ch
          "let %s = foreign \"%s\" (%s @-> returning %s)"
          func
          func
          String.(concat ~sep:" @-> " args)
          rval;
        p ch "\n");
    ps "end"
  in
  Stdio.Out_channel.with_file stubs_filename ~f


let write stubs_filename =
  snd Common.(read_file "mujoco.h") |> parse |> write_stubs ~stubs_filename
