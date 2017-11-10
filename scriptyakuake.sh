INITIAL_ID=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`

#rcnode

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession

sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rcnode'

qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'rcnode'

sleep 1

#rcis

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession

sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rcis ~/robocomp/files/innermodel/simpleworld.xml'

qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'rcis'

sleep 3

#chocachoca

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession

sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd ~/robocomp/components/roboticaCC/chocachoca'

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rm -f CMakeCache.txt'

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake .'

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'make -j4'

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'bin/chocachoca etc/config'

qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'chocachoca'

sleep 1

#supervisor

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession

sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd ~/robocomp/components/roboticaCC/supervisor/'

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rm -f CMakeCache.txt'

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake .'

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'make -j4'

qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'supervisor'

sleep 1


#apriltagsMASTER

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession

sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd ~/robocomp/components/robocomp-robolab/components/apriltagsMASTER'

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rm -f CMakeCache.txt'

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake .'

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'make -j4'

qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'apriltagsMASTER'

sleep 1

echo "Saliendo..."
	COUNTER=10
	while [  $COUNTER -gt 0 ]; do
			 echo $COUNTER
			 let COUNTER=COUNTER-1
			 sleep 1
	done
echo "Saliendo..."

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.removeSession $INITIAL_ID



