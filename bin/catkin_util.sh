TMPDIR=$PWD/.tmp/$$
# /bin/echo "$0 will be working in temporary dir $TMPDIR"

initializeANSI()
{
  esc=""

  blackf="${esc}[30m";   redf="${esc}[31m";    greenf="${esc}[32m"
  yellowf="${esc}[33m"   bluef="${esc}[34m";   purplef="${esc}[35m"
  cyanf="${esc}[36m";    whitef="${esc}[37m"

  blackb="${esc}[40m";   redb="${esc}[41m";    greenb="${esc}[42m"
  yellowb="${esc}[43m"   blueb="${esc}[44m";   purpleb="${esc}[45m"
  cyanb="${esc}[46m";    whiteb="${esc}[47m"

  boldon="${esc}[1m";    boldoff="${esc}[22m"
  italicson="${esc}[3m"; italicsoff="${esc}[23m"
  ulon="${esc}[4m";      uloff="${esc}[24m"
  invon="${esc}[7m";     invoff="${esc}[27m"

  reset="${esc}[0m"
}

initializeANSI

get_version_component()
{
    REGEX='/.*\((\d+)\.(\d+)\.(\d+)\-(\d+)(\w+)\)/'
    NUM=$1
    REV=$2
    VALUE=$(perl -e "\"$REV\" =~ $REGEX  && print \$$NUM")
}

get_upstream_version_component()
{
    REGEX='/upstream\/(\d+)\.(\d+)\.(\d+)/'
    NUM=$1
    REV=$2
    VALUE=$(perl -e "\"$REV\" =~ $REGEX  && print \$$NUM")
}

bailout()
{
    /bin/echo "${redf}${boldon}$*${reset}"
    exit 1
}

checking()
{
    /bin/echo "${yellowf}$*${reset}"
}

status()
{
    /bin/echo "$*"
}

okay()
{
    /bin/echo "${greenf}$*${reset}"
}

github_api()
{
    VARNAME=$1
    shift
    CALL=$1
    shift
    URLS=$(curl -s https://api.github.com/$CALL | $TOP/json-extract $*)
    eval $VARNAME="\"$URLS\""
}

_track_all()
{
    for x in catkin upstream
    do
        if git branch | grep $x >/dev/null
        then
            status "$(basename `pwd`) has branch $x."
        elif git branch -r | grep origin/$x >/dev/null
        then
            git branch --track $x origin/$x
        fi
    done
}
