let config = {
    blocks: [
        {
            name: "Gyro",
            params: [
                "raw_gyro_x",
                "raw_gyro_y",
                "raw_gyro_z",
            ]
        }
    ]
};


class Backend {
    static _PORT = 9090;
    static _API = `http://${document.location.host.split(':')[0]}:${Backend._PORT}/api`;

    /*
        Saves the params state for the given 
    */
    static save(params) {

    }

    static async _fetch(endpoint) {
        let url = `${Backend._API}/${endpoint}`;
        //console.log(`Fetching: ${url}`);
        return fetch(url);
    }

    static async fetchLogBlocks() {
        let result;
        try {
            let res = await Backend._fetch('get_log_blocks');
            result = {
                status: 'OK',
                data: await res.json()
            };
        } catch(error) {
            result = {
                status: 'ERROR',
                data: []
            };
            console.log('Error fetching log blocks!');
        }
        return result;
    }

}

export default Backend;
