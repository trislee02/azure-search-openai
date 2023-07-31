import { Example } from "./Example";

import styles from "./Example.module.css";

export type ExampleModel = {
    text: string;
    value: string;
};

const EXAMPLES: ExampleModel[] = [
    {
        text: "How can I get started with ROS package?",
        value: "How can I get started with ROS package?"
    },
    { text: "Write me Python code to enable the robot to speak", value: "Write me Python code to enable the robot to speak" },
    { text: "Que dois-je faire pour créer un package ROS?", value: "Que dois-je faire pour créer un package ROS?" }
];

interface Props {
    onExampleClicked: (value: string) => void;
}

export const ExampleList = ({ onExampleClicked }: Props) => {
    return (
        <ul className={styles.examplesNavList}>
            {EXAMPLES.map((x, i) => (
                <li key={i}>
                    <Example text={x.text} value={x.value} onClick={onExampleClicked} />
                </li>
            ))}
        </ul>
    );
};
