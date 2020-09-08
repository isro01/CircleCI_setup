for cf in *; do
  cpplint --filter=-whitespace,-legal/copyright,-readability/multiline_comment, --linelength=160 $cf

done

