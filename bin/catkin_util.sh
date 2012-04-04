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

find_workspace() {
    VAR=$1
    pushd .

}

maybe_continue () {
    DEFAULT=$1
    shift
    PROMPT=$*
    # assert_nonempty "$DEFAULT"
    status $PROMPT
    /bin/echo -n "${boldon}Continue "

    if [ "$DEFAULT" = 'y' -o "$DEFAULT" = 'y' ] ; then
        /bin/echo "${yellowf}[Y/n]?${reset}"
    else
        /bin/echo "${yellowf}[y/N]?${reset}"
    fi
    read IN
    if [ -z "$IN" ] ; then
        IN=$DEFAULT
    fi
    if [ $IN = 'N' -o $IN = 'n' ] ; then
        bailout "Exiting."
    fi
}

get_version_component ()
{
    REGEX='/.*\((\d+)\.(\d+)\.(\d+)\-(\d+)(\w+)\)/'
    NUM=$1
    REV=$2
    VALUE=$(perl -e "\"$REV\" =~ $REGEX  && print \$$NUM")
}

get_upstream_version_component ()
{
    REGEX='/upstream\/(\d+)\.(\d+)\.(\d+)/'
    NUM=$1
    REV=$2
    VALUE=$(perl -e "\"$REV\" =~ $REGEX  && print \$$NUM")
}

bailout ()  {
    /bin/echo "${redf}${boldon}$*${reset}"
    exit 1
}

checking ()  {
    /bin/echo "${yellowf}$*${reset}"
}

status () {
    /bin/echo "$*"
}

okay ()  {
    /bin/echo "${greenf}$*${reset}"
}

read_stack_yaml ()
{
    FILENAME=$1
    if [ ! -e $FILENAME ] ; then
        bailout "Hm, attempt to read file $FILENAME which does not exist."
    fi

    TXT=$(cat $FILENAME)

    VERSION_FULL=$(/bin/echo $TXT | perl -ne '/Version:\s+([^\s]+)/ && print $1')
    get_version_component 1 $VERSION_FULL
    VERSION_MAJOR=$VALUE
    get_version_component 2 $VERSION_FULL
    VERSION_MINOR=$VALUE
    get_version_component 3 $VERSION_FULL
    VERSION_PATCH=$VALUE

    PACKAGE_NAME=$(/bin/echo $TXT | perl -ne '/Catkin-ProjectName:\s+([^\s]+)/ && print $1')

    CHECKOUT_TAG=$(/bin/echo $TXT | perl -ne '/Release-Tag:\s+([^\s]+)/ && print $1')
}

prompt_continue()
{
    /bin/echo $*
    status "Press enter to continue, ^C to abort..."
    read MEH
    if [ "$MEH" = 'q' -o "$MEH" = 'Q' ] ; then
        bailout "exiting"
    fi
}

assert_is_remote_git_repo ()
{
    REPO=$1
    checking "Verifying that $REPO is a git repo...${reset}"
    LSREMOT=$(git ls-remote --heads $REPO)
    RCODE=$?
    if [ $RCODE -ne 0 ]; then
        bailout "$REPO doesn't appear to be a git repo!"
    else
        okay "Yup, with $(/bin/echo $LSREMOT | wc -l) heads."
    fi
}

assert_is_gbp_repo ()
{
    REPO=$1
    NDEBTAGS=$(git ls-remote --tags $REPO debian/\* | wc -l)
    checking "Verifying that repo is a git-buildpackage repo"
    if [ $NDEBTAGS -eq 0 ] ; then
        bailout "Repo $REPO doesn't seem to have any tags with 'debian' in them"
    else
        okay "There are $NDEBTAGS debian tags in there. Good."
    fi
    NUPSTREAM=$(git ls-remote --tags $REPO upstream/\* | wc -l)
    checking "Verifying that repo is a git-buildpackage repo"
    if [ $NUPSTREAM -eq 0 ] ; then
        bailout "Repo $REPO doesn't seem to have any tags with 'upstream' in them"
    else
        okay "There are $NUPSTREAM upstream tags.  Good."
    fi
}

assert_is_not_gbp_repo ()
{
    REPO=$1
    LSREMOT=$(git ls-remote --heads $REPO upstream\* | wc -l)
    checking "Verifying that ${boldon}$REPO${boldoff} is ${boldon}not${boldoff} a git-buildpackage repo"
    if [ $LSREMOT -ne 0 ] ; then
        /bin/echo "${redf}Error: $REPO appears to have an 'upstream' branch, but shouldn't, I'm treating it as being the upstream itself."
        git ls-remote --heads $REPO upstream\*
        bailout "Looks like this repo is git-buildpackage, but should not be"
    else
        okay "Yup, no upstream branches."
    fi
}

assert_nonempty ()
{
    if [ -z "$1" ] ; then
        bailout "assertion, failed variable unset"
    fi
}

get_upstream_version_component ()
{
    REGEX='/upstream\/(\d+)\.(\d+)\.(\d+)/'
    NUM=$1
    REV=$2
    VALUE=$(perl -e "\"$REV\" =~ $REGEX  && print \$$NUM")
}


extract_gbp_upstream_version ()
{
    LASTTAG=$1
    get_upstream_version_component 1 "$LASTTAG"
    GBP_MAJOR=$VALUE
    assert_nonempty $GBP_MAJOR
    get_upstream_version_component 2 "$LASTTAG"
    GBP_MINOR=$VALUE
    get_upstream_version_component 3 "$LASTTAG"
    GBP_PATCH=$VALUE
}

to_github_uri ()
{
    SHORTY=$1
    /bin/echo "Having a look at ${!SHORTY}... is it a github url?"

    if [[ "${!SHORTY}" =~ ^git@github.com:wg-debs ]] ; then
        status "URI ${boldon}${!SHORTY}${boldoff} appears to already point to github."
        return 0
    fi
    export TO_GITHUB_TRANSFORM=$(git config catkin.gbproot)
    DEFAULT='git@github.com:wg-debs/$ARG.git'
    if [ -z "$TO_GITHUB_TRANSFORM" ] ; then
        /bin/echo "Your git doesn't have a 'catkin.gbproot' defined."
        /bin/echo "using default of $DEFAULT"
        TO_GITHUB_TRANSFORM=$DEFAULT
    fi
    ARG=${!SHORTY}
    TRANSFORMED=$(eval /bin/echo "$TO_GITHUB_TRANSFORM")
    NEW=$(/bin/echo ${!TO_GITHUB_TRANSFORM})
    status "Redirecting '${!SHORTY}' to ${boldon}$TRANSFORMED${boldoff}"
    eval $SHORTY=$TRANSFORMED
}

check_git_version ()
{
    set -e
    status "Checking git compatibility.  If things bail out your git is too old"
    status "Version 1.7.4.1 is known to work"
    mkdir -p $TMPDIR/gittest
    pushd $TMPDIR/gittest
    git init
    V=compat_test_if_this_fails_your_git_is_too_old
    touch $V
    git add $V
    git commit -m $V
    git checkout --orphan $V
    set +e
    /bin/echo "Your ${boldon}$(git --version)${boldoff} appears to work."
    popd
    rm -rf $TMPDIR/gittest
}

github_api ()
{
    VARNAME=$1
    shift
    CALL=$1
    shift
    URLS=$(curl -s https://api.github.com/$CALL | $TOP/json-extract $*)
    eval $VARNAME="\"$URLS\""
}

github_raw ()
{
    #set -x
    VARNAME=$1
    shift
    CALL=$1
    shift
    TXT=$(curl -s https://raw.github.com/$CALL)
    eval $VARNAME="\"$TXT\""
}
# check_git_version


repo_clone ()
{
    TYPE=$1
    URL=$2
    DEST=$3

    #set -x
    mkdir -p $DEST
    case $TYPE in
        git)
            git clone $URL $DEST
            ;;
        hg)
            hg clone -q $URL $DEST
            ;;
        svn)
            svn co -q $URL $DEST
    esac
    #set +x
}


repo_export ()
{
    #set -x
    TYPE=$1
    REPO=$2
    BASEPATH=$3
    VERSION=$4
    cd $REPO

    case $TYPE in
        git)
            if ! git archive -o $BASEPATH.tar $VERSION ; then
                bailout "unable to archive tag ${yellowf}$VERSION${reset}.  Did you tag the upstream correctly?"
            fi
            gzip $BASEPATH.tar
            ;;
        svn)
            svn export $REPO $BASEPATH
            pushd $BASEPATH
            tar cvzf $BASEPATH.tar.gz .
            popd
            ;;
        hg)
            hg archive -t tar -r $VERSION $BASEPATH.tar
            gzip $BASEPATH.tar
            ;;
        *)
            bailout "What kind of repo is $TYPE?"
            ;;
    esac
}


_track_all(){
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
