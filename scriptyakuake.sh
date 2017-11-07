#rcnode

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession

sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rcnode'

qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'rcnode'

sleep 1

#rcis

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession

sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rcis robocomp/files/innermodel/simpleworld.xml'

qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'rcis'

sleep 3

#chocachoca

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession

sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd robocomp/components/roboticaCC/chocachoca'

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rm -f CMakeCache.txt'

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'make -j4'

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'bin/chocachoca etc/config'

qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'chocachoca'

sleep 1
