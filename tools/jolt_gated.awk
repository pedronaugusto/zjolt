# For each declared name, whether it has a declaration outside upstream's own
# scaffolding macros. SCAFF is the list of macros whose only #define in the
# tree is commented out — they are enabled by editing the source, so nothing
# under them is reachable in any build upstream ships.
#
# Prints NAME<TAB>open  for a declaration outside every scaffolding gate,
#        NAME<TAB>MACRO for one inside it.

BEGIN { n = split(SCAFF, a, " "); for (i = 1; i <= n; i++) scaff[a[i]] = 1 }
FNR == 1 { depth = 0; delete gate; incomment = 0 }

/^[[:space:]]*#[[:space:]]*if/ {
  depth++
  gate[depth] = ""
  if (match($0, /JPH_[A-Z0-9_]+/)) {
    m = substr($0, RSTART, RLENGTH)
    if (m in scaff && $0 !~ /![[:space:]]*defined|#[[:space:]]*ifndef/) gate[depth] = m
  }
  next
}
/^[[:space:]]*#[[:space:]]*else/ { if (depth > 0) gate[depth] = ""; next }
/^[[:space:]]*#[[:space:]]*elif/ {
  if (depth > 0) {
    gate[depth] = ""
    if (match($0, /JPH_[A-Z0-9_]+/)) { m = substr($0, RSTART, RLENGTH); if (m in scaff) gate[depth] = m }
  }
  next
}
/^[[:space:]]*#[[:space:]]*endif/ { if (depth > 0) { gate[depth] = ""; depth-- } ; next }
/^[[:space:]]*#/ { next }

{
  line = $0
  if (incomment) { p = index(line, "*/"); if (p == 0) next; line = substr(line, p+2); incomment = 0 }
  while ((a2 = index(line, "/*")) > 0) {
    rest = substr(line, a2+2); b = index(rest, "*/")
    if (b == 0) { line = substr(line, 1, a2-1); incomment = 1; break }
    line = substr(line, 1, a2-1) substr(rest, b+2)
  }
  sub(/\/\/.*/, "", line)
  $0 = line
}
match($0, /[A-Za-z_][A-Za-z0-9_]*[[:space:]]*\(/) {
  pre = (RSTART > 1) ? substr($0, RSTART-1, 1) : ""
  if (pre == "." || pre == ">" || pre == ":") next
  nm = substr($0, RSTART, RLENGTH-1); gsub(/[[:space:]()]/, "", nm)
  g = ""
  for (i = 1; i <= depth; i++) if (gate[i] != "") { g = gate[i]; break }
  print nm "\t" (g == "" ? "open" : g)
}
