# /bin/sh

if ! type clang-format-3.6;then
  echo "installing clang-format-3.6"
  sudo apt-get install clang-format-3.6
fi

if ! type autopep8;then
  echo "installing autopep8"
  sudo apt-get install python-autopep8
fi


cd $(dirname $0)
find . -type f -regex ".*\.\(c\|cpp\|cxx\|h\|hpp\|hxx\)$" | xargs clang-format-3.6 -i -style=file $1
# find . -type f -regex ".*\.\(c\|cpp\|cxx\|h\|hpp\|hxx\)$" | xargs clang-format-3.6 -i -style="{BasedOnStyle: Google, ColumnLimit: 100, IndentWidth: 2, UseTab: Never}"
find . -type f -regex ".*\.\(py\)$"|while read f;do autopep8 -i --max-line-length 120 --aggressive --aggressive $f;done
