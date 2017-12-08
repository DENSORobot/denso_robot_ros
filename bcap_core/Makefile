PROJECTS = bCAPClient bCAPServer TPComm RACString
COMPILER = Linux

all: projects

projects:
	for prj in $(PROJECTS); do $(MAKE) -C src/$$prj -f Makefile.$(COMPILER) || exit 1; done

install:
	for prj in $(PROJECTS); do $(MAKE) -C src/$$prj -f Makefile.$(COMPILER) install; done

clean:
	for prj in $(PROJECTS); do $(MAKE) -C src/$$prj -f Makefile.$(COMPILER) clean; done
