# Names upstream marks internal, with the marker's FILE:LINE.
#
# Six markers, all Jolt's own words:
#   file    "WARNING: This class is an internal part of PhysicsSystem, it has
#            no functions that can be called by users of the library."
#   class   a class whose doc comment opens "/// Internal ..." — the whole body.
#   section "INTERNAL USE ONLY" — to ///@} if the banner opened a @name group,
#           else to end of file.
#   decl    "/// Internal ..." on the comment immediately above a function.
#   suffix  a name ending in Internal — Body.h:299 says these are "not meant to
#           be called by the application".
#   shout   "INTERNAL CLASS DO NOT USE!" above a class — the whole body.

FNR == 1 { filewide = 0; section = 0; group = 0; pending = 0; incomment = 0
           depth = 0; classdepth = -1 }

/WARNING: This class is an internal part of .*no functions that can be called by users of the library/ {
  filewide = 1; mark = FILENAME ":" FNR; next
}
/INTERNAL USE ONLY|FUNCTIONS BELOW THIS LINE ARE FOR INTERNAL USE/ {
  section = 1; mark = FILENAME ":" FNR; group = ($0 ~ /@name/); next
}
group && /^[[:space:]]*\/\/\/@}/ { section = 0; group = 0; next }

/^[[:space:]]*\/\/\/[[:space:]]*Internal[[:space:]]/ {
  pending = 1; pendmark = FILENAME ":" FNR; next
}
/^[[:space:]]*\/\/\/.*INTERNAL CLASS DO NOT USE/ {
  pending = 1; pendmark = FILENAME ":" FNR; next
}
/^[[:space:]]*\/\/\// { next }

{
  line = $0
  if (incomment) { p = index(line, "*/"); if (p == 0) next; line = substr(line, p+2); incomment = 0 }
  while ((a = index(line, "/*")) > 0) {
    rest = substr(line, a+2); b = index(rest, "*/")
    if (b == 0) { line = substr(line, 1, a-1); incomment = 1; break }
    line = substr(line, 1, a-1) substr(rest, b+2)
  }
  sub(/\/\/.*/, "", line)
  $0 = line
}

# A pending "/// Internal" that lands on a class or struct covers its body.
pending && /^[[:space:]]*(class|struct)[[:space:]]/ && !/;[[:space:]]*$/ {
  classdepth = depth; classmark = pendmark; pending = 0
}

{
  n_open = gsub(/{/, "{"); n_close = gsub(/}/, "}")
  before = depth
  depth += n_open - n_close
  if (classdepth >= 0 && depth <= classdepth && n_close > 0) classdepth = -1
}
!NF { next }

match($0, /[A-Za-z_][A-Za-z0-9_]*[[:space:]]*\(/) {
  pre = (RSTART > 1) ? substr($0, RSTART-1, 1) : ""
  if (pre == "." || pre == ">" || pre == ":") { pending = 0; next }
  n = substr($0, RSTART, RLENGTH-1); gsub(/[[:space:]()]/, "", n)
  if (n ~ /^(if|for|while|switch|return|sizeof|alignof|static_assert|do|else|catch|new|delete|defined|operator|JPH_)/) { pending = 0; next }
  if (n ~ /Internal$/)     print n "\t" FILENAME ":" FNR "\tsuffix"
  else if (filewide)       print n "\t" mark "\tfile"
  else if (section)        print n "\t" mark "\tsection"
  else if (classdepth >= 0) print n "\t" classmark "\tclass"
  else if (pending)        print n "\t" pendmark "\tdecl"
  pending = 0
}
