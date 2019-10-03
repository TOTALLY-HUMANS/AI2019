#!/bin/bash

# loop through challenges
for i in {0..4}
do
    DIR="challenge_$i/submission"

    TEAMNAME="$(cat team_info.txt | head -10 | tail -1)"
    TODAY="$(date +%d_%m)"
    US="_"

    ZIPFILE="submission_$i$US$TEAMNAME$US$TODAY.zip"
    HASHFILE="submit/hash_$i"

    # Check whether the folder challenge_i/submission has some content
    if [ "$(ls -A $DIR 2> /dev/null)" ]; then
        # Submission folder is not empty

        HASH="$(find $DIR -type f -exec cat '{}' \; | md5sum)"
        OLDHASH="$(cat $HASHFILE)"

        # If the hash is stored, the file is already sent:
        # Only send if oldhash != hash
        if [ "$HASH" == "$OLDHASH" ]; then
            echo "These files for challenge $i have already been submitted"
            echo ""
        else
            echo "Compressing submission for challenge $i!"

            # Zip the submission with the team info
            zip -rq "$ZIPFILE" "$DIR" "team_info.txt"

            # Save the hash into a gitlab CI artifact
            echo "$HASH" > $HASHFILE

            SIZE="$(wc -c < $ZIPFILE)"

            # Check file size, don't send if too large (8 MB)
            if [ $SIZE -lt 8388608 ]; then

                # Send the file
                curl -F "file=@$ZIPFILE"  http://lab.robotuprising.fi:8080/upload
                echo ""
                echo "Challenge $i submitted!"
                echo ""
            else
                echo "Submission size must be less than 8 MB!"
            fi
        fi
    else
        echo "No submission for challenge $i"
    fi
done

