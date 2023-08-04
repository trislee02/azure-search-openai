import { renderToStaticMarkup, renderToString } from "react-dom/server";
import { getCitationFilePath } from "../../api";

type HtmlParsedAnswer = {
    answerHtml: string;
    citations: string[];
    followupQuestions: string[];
};

export function parseAnswerToHtml(answer: string, onCitationClicked: (citationFilePath: string) => void): HtmlParsedAnswer {
    const citations: string[] = [];
    const followupQuestions: string[] = [];

    // Extract any follow-up questions that might be in the answer
    let parsedAnswer = answer.replace(/<<([^>>]+)>>/g, (match, content) => {
        followupQuestions.push(content);
        return "";
    });

    // trim any whitespace from the end of the answer after removing follow-up questions
    parsedAnswer = parsedAnswer.trim();

    // const parts = parsedAnswer.split(/\[([^\]]+)\]/g);
    const parts = parsedAnswer.split(/(\[([^\]]+)\])|(```.*?```)/gs);

    const fragments: string[] = parts.map((part, index) => {
        console.log(part);
        if (part && part[0] === "[") {
            let citationIndex: number;
            var refinedPart = part.match(/(?<=\[)(.*?)(?=\])/g);
            var source = "";
            if (refinedPart) source = refinedPart[0];

            if (citations.indexOf(source) !== -1) {
                citationIndex = citations.indexOf(source) + 1;
            } else {
                citations.push(source);
                citationIndex = citations.length;
            }

            const path = getCitationFilePath(source);

            return renderToStaticMarkup(
                <a className="supContainer" title={source} onClick={() => onCitationClicked(path)}>
                    <sup>{citationIndex}</sup>
                </a>
            );
        } else if (part && part[0] === "`") {
            return renderToStaticMarkup(
                <pre>
                    <code>{part}</code>
                </pre>
            );
        } else {
            return part;
        }
    });

    return {
        answerHtml: fragments.join(""),
        citations,
        followupQuestions
    };
}
