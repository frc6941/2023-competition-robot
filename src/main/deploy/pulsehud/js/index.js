import { NT4_Client } from '../js/NT4.js'

const matchTimeTopic = "/matchInfo/matchTime"
const cursorTopic = "/TargetSelector/cursor"
const targetTopic = "/TargetSelector/target"

const { createApp } = Vue


let app = createApp({
    data() {
        return {
            matchTime: "-1.0",
            matchCode: "PRAC",
            matchName: "练习",
            period: "准备阶段",
            isConnected: false,
            allChecked: false,
            isDSAttached: false,

            target: [2, 1],
            cursor: [2, 2],
        }
    },
    methods: {
        connected() {
            this.isConnected = true
        },
        disconnected() {
            this.isConnected = false
        },
        handleTopic(topic, value) {
            if(topic == matchInfoTopic) {

            } else if (topic == cursorTopic) {
                this.cursor = value
            } else if (topic == targetTopic) {
                this.target = value
            }
        },
        isActive(row, column) {
            return row == this.target[0] && column == 8 - this.target[1]
        },
        isCursor(row, column) {
            return row == this.cursor[0] && column == 8 - this.cursor[1]
        },
        isCube(row, column) {
            if (row == 0) {
                return true
            } else {
                if (column == 1 || column == 4 || column == 7) {
                    return true
                }
                return false
            }
        },
        isSeparation(column) {
            return column == 2 || column == 5 
        },
        nodes(row) {
            let result = []
            for (let i = 0; i < 9; i++) {
                let temp = ""
                if (this.isCube(row, i)) {
                    temp = "box cube"
                } else {
                    temp = "box cone"
                }
                if (this.isActive(row, i)) {
                    temp += " active"
                }
                if (this.isCursor(row, i)) {
                    temp += " cursor"
                }

                result.push(temp)
                if (this.isSeparation(i)) {
                    result.push("box separation")
                }
            }
            return result
        },
    },
    computed: {
        hybridNodes() {
            return this.nodes(0)
        },
        midNodes() {
            return this.nodes(1)
        },
        highNodes() {
            return this.nodes(2)
        }
    },
    mounted() {
        let client = new NT4_Client(
            window.location.hostname,
            "TargetSelector",
            (topic) => {
                // Topic announce
            },
            (topic) => {
                // Topic unannounce
            },
            (topic, timestamp, value) => {
            },
            () => {
                this.connected()
            },
            () => {
                this.disconnected()
            }
        )
        client.connect()
    }
})


app.mount('#app')
