import { NT4_Client } from '../js/NT4.js'

const matchTimeTopic = "/MatchInfo/matchTime"
const matchTypeTopic = "/MatchInfo/matchType"
const matchNameTopic = "/MatchInfo/matchName"
const matchNumberTopic = "/MatchInfo/matchNumber"
const matchStageTopic = "/MatchInfo/matchStage"
const dsConnectedTopic = "/MatchInfo/dsConnected"

const cursorTopic = "/TargetSelector/cursor"
const targetTopic = "/TargetSelector/target"

const targetCollection = [
    matchTimeTopic, matchTypeTopic, matchNameTopic, matchNumberTopic, matchStageTopic, dsConnectedTopic,
    cursorTopic, targetTopic
]

const stageMap = new Map()
stageMap.set("PREP", "准备阶段")
stageMap.set("AUTO", "自动阶段")
stageMap.set("TELEOP", "手动阶段")
stageMap.set("ENDGAME", "终局")
stageMap.set("ESTOP", "急停")
stageMap.set("DISCONNECTED", "未连接")

const bannerClassPrefix = "hero is-small "
const bannerMap = new Map()
bannerMap.set("PREP", "is-info")
bannerMap.set("AUTO", "is-link")
bannerMap.set("TELEOP", "is-info")
bannerMap.set("ENDGAME", "is-warning")
bannerMap.set("ESTOP", "is-danger")
bannerMap.set("DISCONNECTED", "is-disconnected")


const { createApp } = Vue


let app = createApp({
    data() {
        return {
            matchTime: "-1.0",
            matchType: "None",
            matchNumber: 0,
            matchName: "练习",
            matchStage: "准备阶段",
            isConnected: false,
            allChecked: false,
            dsConnected: false,

            target: [2, 1],
            cursor: [2, 2],

            bannerClass: bannerClassPrefix + "is-disconnected"
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
            if(topic == matchTimeTopic) {
                this.matchTime = value.toFixed(1)
            } else if(topic == matchTypeTopic) {
                this.matchType = value
            } else if(topic == matchNameTopic) {
                this.matchName = value
            } else if(topic == matchNumberTopic) {
                this.matchNumber = value.toFixed(0)
            } else if(topic == matchStageTopic) {
                this.matchStage = stageMap.get(value)
                this.bannerClass = bannerClassPrefix + bannerMap.get(value)
            } else if(topic == dsConnectedTopic) {
                this.dsConnected = value
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
            console.log(this.cursor)
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
            "PulseHUD",
            (topic) => {
                // Topic announce
            },
            (topic) => {
                // Topic unannounce
            },
            (topic, timestamp, value) => {
                this.handleTopic(topic.name, value)
            },
            () => {
                this.connected()
            },
            () => {
                this.disconnected()
            }
        )

        client.subscribe(
            targetCollection,
            false,
            false,
            0.02
        );
        client.connect()
    }
})


app.mount('#app')
