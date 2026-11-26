import json
from typing import Any

from langchain_core.messages import HumanMessage, SystemMessage
from langchain_groq import ChatGroq

from .orchestrator import create_orchestrator


ALLOWED_ACTIONS = {"explorer", "advancer", "photographer", "end"}


def create_orchestrator_llm(
    *,
    max_cycles: int,
    max_duration_seconds: float,
    min_photo_interval_seconds: float,
    minimum_clearance_cm: float,
    obstacle_clearance_cm: float,
    obstacle_safety_margin_cm: float,
    advance_fraction: float,
    maximum_advance_distance_cm: float,
    minimum_advance_duration_seconds: float,
    forward_speed_mps: float,
    model: str,
):
    """Use Groq to choose a safe next action from the three agent reports."""
    llm = ChatGroq(model=model, temperature=0)
    deterministic = create_orchestrator(
        max_cycles=max_cycles,
        max_duration_seconds=max_duration_seconds,
        min_photo_interval_seconds=min_photo_interval_seconds,
        minimum_clearance_cm=minimum_clearance_cm,
        obstacle_clearance_cm=obstacle_clearance_cm,
        obstacle_safety_margin_cm=obstacle_safety_margin_cm,
        advance_fraction=advance_fraction,
        maximum_advance_distance_cm=maximum_advance_distance_cm,
        minimum_advance_duration_seconds=minimum_advance_duration_seconds,
        forward_speed_mps=forward_speed_mps,
    )

    async def orchestrator_llm(state: dict[str, Any]) -> dict[str, Any]:
        safe_default = await deterministic(state)
        if (
            state["last_agent"] != "explorer"
            or safe_default["next_action"] == "end"
        ):
            return safe_default

        context = {
            "next_action": safe_default["next_action"],
            "selected_direction": state["selected_direction"],
            "distance": state["distance"],
            "previous_distance": state["previous_distance"],
            "selected_clearance_cm": state["selected_clearance_cm"],
            "readings": state["readings"],
            "agent_results": state["agent_results"],
        }
        messages = [
            SystemMessage(
                content=(
                    "You are a navigation orchestrator for a small robot. "
                    "Use the Explorer, Advancer, and Photographer reports in the "
                    "context. Choose next_action from explorer, advancer, "
                    "photographer, or end. Choose advancer only when the selected "
                    "direction has sufficient clearance. If there is no safe "
                    "direction, choose end. The Advancer only moves forward by the "
                    "distance already requested; never output motor commands or "
                    "invent sensor readings. Respond with one JSON object containing "
                    '{"next_action":"...","reason":"..."}'
                )
            ),
            HumanMessage(content=json.dumps(context, ensure_ascii=False)),
        ]

        try:
            response = await llm.ainvoke(messages)
            content = response.content
            if isinstance(content, list):
                content = "".join(
                    part.get("text", "") if isinstance(part, dict) else str(part)
                    for part in content
                )
            decision = json.loads(str(content).strip())
            action = decision.get("next_action")
            reason = decision.get("reason")
            if action not in ALLOWED_ACTIONS:
                raise ValueError(f"Invalid next_action: {action!r}")
            if not isinstance(reason, str) or not reason.strip():
                reason = "The LLM selected the next navigation action."

            if (
                action == "photographer"
                and safe_default["next_action"] != "photographer"
            ):
                print("LLM decision rejected: photo interval has not elapsed.")
                return safe_default
            if action == "advancer" and (
                state["selected_direction"] is None
                or state["selected_clearance_cm"] is None
                or state["selected_clearance_cm"] <= minimum_clearance_cm
            ):
                print("LLM decision rejected: selected direction is not safe.")
                return safe_default

            print(f"LLM orchestrator: {action}. Reason: {reason}")
            updates = {"next_action": action, "reason": reason}
            if action == "end":
                updates["status"] = "LLM_STOP"
            return {**safe_default, **updates}
        except Exception as exc:
            print(
                f"LLM decision failed; using deterministic routing: "
                f"{type(exc).__name__}: {exc}"
            )
            return safe_default

    return orchestrator_llm
