#!/bin/bash

##
## Copyright (c) 2024 Numurus <https://www.numurus.com>.
##
## This file is part of nepi setup tools (nepi_setup) repo
## (see https://github.com/nepi-engine/nepi_setup)
##
## License: nepi setup tools are licensed under the "Numurus Software License",
## which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
##
## Redistributions in source code must retain this top-level comment block,
## Along with any License Check related code and checks.
## Plagiarizing this software to sidestep the license obligations is illegal.
##
## Contact Information:
## ====================
## - mailto:nepi@numurus.com
##

# This file Checks if the Numurus Software License has been accepted

SCRIPT_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
USER_CHECK_FILE=${SCRIPT_FOLDER}/nepi_user_check.sh
source $USER_CHECK_FILE
if [[ "$?" -ne 0 ]]; then
    return
fi

SCRIPT_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)


####################################
# Run NEPI Bash Setup Script

NEPI_LICENSE_SOURCE=$(dirname "${SCRIPT_FOLDER}")/LICENSE
NEPI_LICENSE_DEST=/home/${CONFIG_USER}/NEPI_SOFTWARE_LICENSE

license_accepted=no
if [[ ! -f "$NEPI_LICENSE_DEST" ]]; then
    echo ""
    echo "---------------------------------------------------"
    echo "NEPI Setup Tools along with NEPI Image files"
    echo "either installed or created with these NEPI Setup Tools"
    echo "are licensed under the 'Numurus Software License' terms."
    echo ""
    echo "The Numurus Software License is available at:"
    echo "https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf"
    echo "---------------------------------------------------"
    echo ""
    echo "This license allows for the personal use of NEPI Setup Tools and NEPI Image files"
    echo "for trial purposes, but requires either an Educational License if used"
    echo "in a classroom or research environment, or a Professional Licnese if used in either"
    echo "a commercial product or service"
    echo ""
    echo "You must accept the license terms stated above before continuing"
    echo ""
    echo "DO YOU ACCEPT the Numurus Software License Terms?"
    license_accepted=$(ask_yes_no)
    if [[ "$license_accepted" == 'no' ]]; then
        echo ""
        echo "License Terms NOT Accepted, Exiting"
        return 1
    else
        echo ""
        echo "License Terms Accepted"
        echo ""
        echo ""
        sudo cp $NEPI_LICENSE_SOURCE $NEPI_LICENSE_DEST
        return 0
    fi
fi
