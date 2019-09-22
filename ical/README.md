# iCal

## Changelog
1.5.2:
- Use domain name as filename if no alias is defined
- Parse calendars in plugin.yaml more robust

1.5.1:
- Fix reading offline files and line breaks
- Updated code
- using network library instead of fetch_url to download online calendars into var/ical
- possibility to disable https verification when using online calendars
- user documentation
- updated plugin.yaml
- new parameters: directory and timeout to configure download of online calendars
- implement a "prio" attribute to choose which information in an entry should be used
