# PickupRouter

Oftentimes for waterskiing I am picking people up and dropping them off, and it would be nice to know the optimal order to do this.

I wrote this little tool that given the addresses of everyone and our destination, it will tell me what the order gives the least travel time.

It uses the google distance matrix api and google-or tools to determine travel times and find the optimal route.

Note that there are two files not in the repository needed to run this - secrets.py and inputs.py. These hold my google api key and the people's addresses that are getting picked up.
