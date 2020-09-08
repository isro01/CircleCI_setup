for cf in *; do
    cpplint --filter=-legal/copyright,-readability/multiline_comment,-readability/braces,-build/include_order,-build/c++11,-build/include_what_you_use,-runtime/string,-whitespace/indent,+build/namespace,+readability/constructors --linelength=160 $cf

done

