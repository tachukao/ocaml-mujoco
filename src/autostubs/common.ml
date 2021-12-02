open Base

let to_ctypes x =
  match String.(strip x) with
  | "unsigned char" -> "uchar"
  | x               -> x


let convert_typ s =
  if String.(strip s = "char*")
  then "string"
  else (
    match String.split ~on:'*' s with
    | []       -> failwith "Split string should not reach here"
    | [ hd ]   -> to_ctypes hd
    | hd :: tl ->
      let n = List.length tl in
      let left = List.init n ~f:(fun _ -> "(ptr ") |> String.concat ~sep:"" in
      let right = List.init n ~f:(fun _ -> ")") |> String.concat ~sep:"" in
      Printf.sprintf "%s%s%s" left (to_ctypes hd) right)
