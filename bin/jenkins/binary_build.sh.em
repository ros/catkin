#!/bin/bash
ROS_REPO_FQDN=@(FQDN)
ROS_PACKAGE_REPO=@(ROS_PACKAGE_REPO)
PACKAGE=@(PACKAGE)
ROS_DISTRO=@(ROS_DISTRO)
distro=@(DISTRO)
arch=@(ARCH)
DEBPACKAGE=ros-$ROS_DISTRO-@(PACKAGE.replace('_','-'))
base=/var/cache/pbuilder-$ROS_DISTRO-$distro-$arch
aptconfdir=$base/apt-conf
aptstatedir=$base/apt-state
basetgz=$base/base.tgz
output_dir=$WORKSPACE/output
work_dir=$WORKSPACE/work

sudo apt-get update
sudo apt-get install -y pbuilder

sudo mkdir -p $aptconfdir
sudo mkdir -p $aptconfdir/apt.conf.d
sudo mkdir -p $aptconfdir/preferences.d
sudo mkdir -p $aptstatedir/lists/partial


sudo rm -rf $output_dir
mkdir -p $output_dir

sudo rm -rf $work_dir
mkdir -p $work_dir
cd $work_dir

echo "
deb http://archive.ubuntu.com/ubuntu $distro main restricted universe multiverse
deb $ROS_PACKAGE_REPO $distro main
deb-src $ROS_PACKAGE_REPO $distro main
" > sources.list
sudo cp sources.list $aptconfdir

if [ ! -e $basetgz ]
then
  sudo pbuilder create \
    --distribution $distro \
    --aptconfdir $aptconfdir \
    --basetgz $basetgz \
    --architecture $arch 
else
  sudo pbuilder --update --basetgz $basetgz
fi

echo "
Dir::Etc $aptconfdir;
Dir::State $aptstatedir;
" > apt.conf

sudo apt-get update -c $work_dir/apt.conf
sudo apt-get source $DEBPACKAGE -c $work_dir/apt.conf

mkdir -p hooks

echo "#!/bin/bash -ex
echo \`env\`
cd /tmp/buildd/*/
apt-get install devscripts -y
prevversion=\`dpkg-parsechangelog | grep Version | awk '{print \$2}'\`
debchange -v \$prevversion-\`date +%Y%m%d-%H%M-%z\` 'Time stamping.'
cat debian/changelog
" >> hooks/A50stamp
chmod +x hooks/A50stamp

#  --binary-arch even if "any" type debs produce arch specific debs
sudo pbuilder  --build \
    --basetgz $basetgz \
    --buildresult $output_dir \
    --debbuildopts \"-b\" \
    --hookdir hooks \
    *.dsc

/bin/echo "Test.  You may delete this line."

echo "
[debtarget]
method                  = scp
fqdn                    = $ROS_REPO_FQDN
incoming                = /var/www/repos/building/queue/$distro
run_dinstall            = 0
post_upload_command     = ssh rosbuild@@$ROS_REPO_FQDN -- /usr/bin/reprepro -b /var/www/repos/building --ignore=emptyfilenamepart -V processincoming $distro 
" > $output_dir/dput.cf

# invalidate all binary packages which will depend on this package
# listing for now, change to removefilter when confident its working right
#ssh rosbuild@@$ROS_REPO_FQDN -- /usr/bin/reprepro -b /var/www/repos/building -T deb -V listfilter $distro 'Package (% ros-* ), Depends (% *ros-$ROS_DISTRO-$PACKAGE* )'

# push the new deb
dput -u -c $output_dir/dput.cf debtarget $output_dir/*$DISTRO*.changes
