# For every declaration of a name, whether the scope it sits in is public.
#
# Namespace scope is public: a free function needs no access specifier. Inside
# a class or struct the current specifier decides, and the specifier is scoped
# to that class — so the depth of each class body is tracked and popped, rather
# than letting one header's last `private:` leak into everything after it.
#
# Every enclosing scope has to be public, not just the innermost: a public
# member of a privately nested class is no more reachable than a private one.
# CharacterVirtual's ContactCollector is the case that proves it.
#
# A name with no public declaration anywhere cannot be called by a C++ host
# either, so INTERNAL is true by construction rather than by anyone's judgement.

FNR == 1 { depth = 0; top = 0; incomment = 0; pending = 0 }

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

/^[[:space:]]*#/ { next }

# A class or struct opens a new access scope at the depth its body starts on.
/(^|[^A-Za-z0-9_])(class|struct|union)[[:space:]]+[A-Za-z_]/ && !/;[[:space:]]*$/ {
  pending = /(^|[^A-Za-z0-9_])(struct|union)[[:space:]]/ ? 1 : 0
  pendopen = 1
}

/^[[:space:]]*(private|protected):/ { if (top > 0) acc[top] = 0; next }
/^[[:space:]]*public:/              { if (top > 0) acc[top] = 1; next }

{
  n_open = gsub(/{/, "{"); n_close = gsub(/}/, "}")
  for (i = 0; i < n_open; i++) {
    depth++
    if (pendopen) { top++; scope[top] = depth; acc[top] = pending; pendopen = 0 }
  }
  for (i = 0; i < n_close; i++) {
    if (top > 0 && scope[top] == depth) top--
    depth--
  }
}

!NF { next }

match($0, /[A-Za-z_][A-Za-z0-9_]*[[:space:]]*\(/) {
  pre = (RSTART > 1) ? substr($0, RSTART-1, 1) : ""
  if (pre == "." || pre == ">" || pre == ":") next
  n = substr($0, RSTART, RLENGTH-1); gsub(/[[:space:]()]/, "", n)
  pub = 1
  for (i = 1; i <= top; i++) if (!acc[i]) { pub = 0; break }
  print n "\t" (pub ? "public" : "nonpublic")
}
