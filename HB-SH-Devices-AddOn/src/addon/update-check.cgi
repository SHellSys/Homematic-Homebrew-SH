#!/bin/tclsh
set checkURL    "https://github.com/SHellSys/Homematic-Homebrew-SH/tree/main/HB-SH-Devices-AddOn/src/addon/VERSION"
set downloadURL "https://github.com/SHellSys/Homematic-Homebrew-SH/tree/main/hb-sh-devices-addon.tgz"

catch {
  set input $env(QUERY_STRING)
  set pairs [split $input &]
  foreach pair $pairs {
    if {0 != [regexp "^(\[^=]*)=(.*)$" $pair dummy varname val]} {
      set $varname $val
    }
  }
}

if { [info exists cmd ] && $cmd == "download"} {
  puts -nonewline "Content-Type: text/html; charset=utf-8\r\n\r\n"
  puts -nonewline "<html><head><meta http-equiv='refresh' content='0; url=$downloadURL' /></head><body></body></html>"
} else {
  puts -nonewline "Content-Type: text/plain; charset=utf-8\r\n\r\n"
  catch {
    set newversion [ exec /usr/bin/wget -qO- --no-check-certificate $checkURL ]
  }
  if { [info exists newversion] } {
    puts $newversion
  } else {
    puts "n/a"
  }
}