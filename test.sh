# for cf in *; do
#   cpplint --filter=-whitespace,-legal/copyright,-readability/multiline_comment, --linelength=160 $cf

# done

# variable = $(git diff --name-only HEAD~1)

# var=$(git diff --name-only HEAD~1)
# echo $var


# OIFS=$IFS
# IFS='/'
# mails2=${var[2]}
# for x in $mails2
# do
#     if [[ "$x" == *"iarc"* ]]; then
#         echo "> [$x]"
#         pkgs=("$x", )
#     fi
# done

# IFS=$OIFS

# A="$(cut -d'/' -f1 <<<"${var[2]}")"
# echo "$A"

declare -a var
var=($(git diff --name-only HEAD~4))
# echo ${var[2]}

len=${#var[@]}
str="a"
echo "$str"
for((i=0;i<len;i++)); do
    cpplint --filter=-legal/copyright,-readability/multiline_comment,-readability/braces,-build/include_order,-build/c++11,-build/include_what_you_use,-runtime/string,-whitespace/indent,-whitespace/comments,+build/namespace,+readability/constructors --linelength=160 ${var[$i]}

    A="$(cut -d'/' -f1 <<<"${var[$i]}")"
    if [[ "$A" == *"iarc"* && "$A" != "$str" ]]; then
        # if [[ "$A" != "$str" ]]; then
            echo "$A"
            str="$A"
        # fi
    fi
done
